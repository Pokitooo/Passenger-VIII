#ifndef ARDUINO_EXTENDED_H
#define ARDUINO_EXTENDED_H

#include <type_traits>
#include <concepts>
#include <utility>
#include <cmath>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "lib_xcore"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)     \
  ((byte) & 0x80 ? '1' : '0'),   \
    ((byte) & 0x40 ? '1' : '0'), \
    ((byte) & 0x20 ? '1' : '0'), \
    ((byte) & 0x10 ? '1' : '0'), \
    ((byte) & 0x08 ? '1' : '0'), \
    ((byte) & 0x04 ? '1' : '0'), \
    ((byte) & 0x02 ? '1' : '0'), \
    ((byte) & 0x01 ? '1' : '0')

namespace traits {
  template<typename S>
  concept has_ostream = requires(S s, const char *str) {
    { s << str } -> std::same_as<S &>;
  };

  template<typename S>
  concept arduino_stream_derived = std::derived_from<S, Stream>;

  template<typename S>
  concept has_flush = requires(S s) {
    { s.flush() } -> std::same_as<void>;
  };

  template<typename S, typename... Ts>
  concept has_print = requires(S s, Ts... vs) {
    { s.print(vs...) };
  };

  template<typename S>
  concept ostream_flush = has_ostream<S> && has_flush<S>;
}  // namespace traits

namespace stream {
  inline const char *crlf = "\r\n";
  inline const char *lf   = "\n";

  namespace detail {
    class flush_type {};
  }  // namespace detail

  constexpr detail::flush_type flush = detail::flush_type();
}  // namespace stream

template<typename S, typename T>
  requires traits::arduino_stream_derived<S> || traits::has_print<S, T>
S &operator<<(S &stream, T &&v) {
  stream.print(xcore::forward<T>(v));
  return stream;
}

template<typename T>
String &operator<<(String &string, T &&v) {
  string += xcore::forward<T>(v);
  return string;
}

template<typename... IoTypes>
struct IoHook {
private:
  xcore::tuple<IoTypes &...> serials_;  // Store references to Serial objects

public:
  explicit IoHook(IoTypes &...serials) : serials_(serials...) {}

  template<typename Arg>
  IoHook &operator<<(Arg &&arg) {
    forEach([&](auto &serial) { serial << xcore::forward<Arg>(arg); });
    return *this;
  }

private:
  template<typename Func, size_t Index = 0>
  void forEach(Func &&func) {
    if constexpr (Index < sizeof...(IoTypes)) {
      func(xcore::get<Index>(serials_));
      forEach<Func, Index + 1>(xcore::forward<Func>(func));
    }
  }
};

struct FakeOStream {
  template<typename T>
  constexpr FakeOStream &operator<<(T &&) { return *this; }
};

namespace detail {
  template<typename T>
    requires traits::has_ostream<T>
  struct flush_ostream {
    static constexpr bool value = false;
  };

  template<traits::ostream_flush T>
  struct flush_ostream<T> {
    static constexpr bool value = true;
  };

  template<traits::has_ostream OStream,
           size_t              ReserveSize = 0,
           bool                NewLine     = false,
           bool                AutoFlush   = true>
  class csv_stream {
  private:
    OStream *m_stream = {};
    String   m_string = {};

  public:
    explicit csv_stream(OStream &stream)
        : m_stream(&stream) {
      m_string.reserve(ReserveSize);
    }

    csv_stream(const csv_stream &other)     = delete;
    csv_stream(csv_stream &&other) noexcept = delete;

    template<typename T>
    csv_stream &operator<<(T &&value) {
      m_string += xcore::forward<T>(value);
      m_string += ",";
      return *this;
    }

    ~csv_stream() {
      // End of message

      // Remove trailing comma
      m_string.remove(m_string.length() - 1);

      if constexpr (NewLine) {
        m_string += stream::crlf;
      }

      // Flush to stream
      *m_stream << m_string;

      if constexpr (AutoFlush && flush_ostream<OStream>::value) {
        m_stream->flush();
      }
    }
  };
}  // namespace detail

/**
 * @tparam OStream Output Stream Type with "<<" stream operator
 * @tparam ReserveSize Internal string reserve size
 * @tparam NewLine Whether to insert CRLF at the end or not
 * @param stream Ouptut stream OStream object
 * @return Csv stream object
 */
template<traits::has_ostream OStream, size_t ReserveSize = 0, bool NewLine = false, bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush> csv_stream(OStream &stream) {
  return detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>(stream);
}

template<traits::has_ostream OStream, size_t ReserveSize = 0, bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, true, AutoFlush> csv_stream_crlf(OStream &stream) {
  return csv_stream<OStream, ReserveSize, true, AutoFlush>(stream);
}

// I2C Scanner
inline void i2c_detect(Stream       &output_stream,
                       TwoWire      &i2c_wire,
                       const uint8_t addr_from,
                       const uint8_t addr_to) {
  char buf[10];
  output_stream.println("I2C Detector");
  output_stream.print("   ");
  for (uint8_t i = 0; i < 16; i++) {
    sprintf(buf, "%3x", i);
    output_stream.print(buf);
  }

  for (uint8_t addr = 0; addr < 127; addr++) {
    if (addr % 16 == 0) {
      sprintf(buf, "\n%02x:", addr & 0xF0);
      output_stream.print(buf);
    }
    if (addr >= addr_from && addr <= addr_to) {
      i2c_wire.beginTransmission(addr);
      if (const uint8_t resp = i2c_wire.endTransmission(); resp == 0) {
        // device found
        //stream.printf(" %02x", addr);
        sprintf(buf, " %02x", addr);
        output_stream.print(buf);
      } else if (resp == 4) {
        // other resp
        output_stream.print(" XX");
      } else {
        // resp = 2: received NACK on transmit of addr
        // resp = 3: received NACK on transmit of data
        output_stream.print(" --");
      }
    } else {
      // addr not scanned
      output_stream.print("   ");
    }
  }
  output_stream.println("\n");
}

inline double pressure_altitude(const double pressure_hpa) {
  constexpr double h0      = 44307.69396;
  constexpr double p0      = 1013.25;
  const double     h_ratio = pressure_hpa / p0;
  const double     v       = 1 - std::pow(h_ratio, 0.190284);
  return h0 * v;
}

constexpr uint8_t *byte_cast(void *ptr) {
  return static_cast<uint8_t *>(ptr);
}

inline __attribute__((__always_inline__)) void do_nothing() {}

#endif  // ARDUINO_EXTENDED_H