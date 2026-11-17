#ifndef PROBE_MATCHER_H
#define PROBE_MATCHER_H

#include <cstdint>

/// One frame on the wire and the GPIO edge that answered it (times in cycles).
struct Result {
    bool matched;     // false: no GPIO edge within the timeout
    uint32_t index;   // frame number since reset
    uint64_t start;   // first edge of the frame
    uint64_t end;     // last edge of the frame
    uint64_t gpio;    // GPIO edge, 0 if !matched
    uint32_t edges;   // edges in the frame
};

/// Pairs each frame on the wire with the first GPIO edge after its end.
/// Feed frames and edges in time order per channel, then call flush().
class Matcher {
public:
    struct Params {
        uint64_t min_frame;  // shorter activity = link pulse, dropped
        uint64_t timeout;    // a frame without a GPIO edge by then is unmatched
    };

    explicit Matcher(const Params& p) : p_(p) {}

    /// A burst on the wire; shorter than min_frame = link pulse, dropped.
    void frame(uint64_t start, uint64_t end, uint32_t edges)
    {
        if (end - start < p_.min_frame) {
            link_pulses = link_pulses + 1;
            return;
        }
        if (!frames_.push(Frame{start, end, edges})) {
            dropped = dropped + 1;
        }
    }

    /// Starts a new run at @p now: frames that started earlier are not
    /// reported, numbering restarts at 0. Pending frames and edges are kept,
    /// so a frame in progress keeps its GPIO edge.
    void reset(uint64_t now)
    {
        epoch_ = now;
        next_index_ = 0;
        link_pulses = 0;
        stray_edges = 0;
        dropped = 0;
    }

    void gpio_edge(uint64_t t)
    {
        if (!gpio_.push(t)) {
            dropped = dropped + 1;
        }
    }

    /// Emits results decided before @p safe: no frame can still end before it
    /// (start of the burst in progress, else now - gap).
    template <typename Out>
    void flush(uint64_t safe, Out&& out)
    {
        uint64_t g;
        while (gpio_.front(g) && g < safe) {
            gpio_.pop();
            // a later frame also ended before g: the earlier one got no edge
            Frame f;
            while (frames_.second(f) && f.end <= g) {
                frames_.front(f);
                emit_unmatched(f, out);
            }
            if (frames_.front(f) && f.end <= g) {
                frames_.pop();
                emit(Result{true, 0, f.start, f.end, g, f.edges}, out);
            } else if (g >= epoch_) {
                stray_edges = stray_edges + 1;  // GPIO edge without a frame before it
            }
        }
        Frame f;
        while (frames_.front(f) && f.end + p_.timeout < safe) {
            emit_unmatched(f, out);
        }
    }

    // read by the other core for the summary (written only by this one)
    volatile uint32_t link_pulses = 0;
    volatile uint32_t stray_edges = 0;
    volatile uint32_t dropped = 0;  // internal queues full

private:
    struct Frame {
        uint64_t start;
        uint64_t end;
        uint32_t edges;
    };

    template <typename T, uint32_t N>
    struct Fifo {
        T items[N];
        uint32_t head = 0;
        uint32_t tail = 0;

        bool push(const T& v)
        {
            if (tail - head == N) {
                return false;
            }
            items[tail++ % N] = v;
            return true;
        }
        bool front(T& v) const
        {
            if (tail == head) {
                return false;
            }
            v = items[head % N];
            return true;
        }
        bool second(T& v) const
        {
            if (tail - head < 2) {
                return false;
            }
            v = items[(head + 1) % N];
            return true;
        }
        void pop() { ++head; }
    };

    /// @p f is the front frame
    template <typename Out>
    void emit_unmatched(const Frame& f, Out&& out)
    {
        frames_.pop();
        emit(Result{false, 0, f.start, f.end, 0, f.edges}, out);
    }

    /// Numbers the result, or drops it if its frame started before reset().
    template <typename Out>
    void emit(Result r, Out&& out)
    {
        if (r.start < epoch_) {
            return;
        }
        r.index = next_index_++;
        out(r);
    }

    Params p_;
    uint64_t epoch_ = 0;
    uint32_t next_index_ = 0;
    Fifo<Frame, 32> frames_;
    Fifo<uint64_t, 64> gpio_;
};

#endif // PROBE_MATCHER_H
