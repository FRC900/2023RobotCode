// Classes to create CUDA-event based timing for profiling
#ifndef CUDA_EVENT_TIMING_INC__
#define CUDA_EVENT_TIMING_INC__
#include <iosfwd>          // for ostream
#include <map>             // for map
#include <string>          // for string
#include "driver_types.h"  // for cudaStream_t, CUevent_st, cudaEvent_t

// And individual timing event between two points in the code
// Users call start() and end() to mark the start and end of the event
// to time. The event is recorded on the provided CUDA stream.
class Timing
{
public:
    explicit Timing(const std::string &name);

    Timing(const Timing &other) = delete;
    Timing(Timing &&other) noexcept = default;

    Timing &operator=(const Timing &other) = delete;
    Timing &operator=(Timing &&other) noexcept = default;

    virtual ~Timing();
   
    void start(cudaStream_t cudaStream);
    void end(void);
    void endFrame(void);
    friend std::ostream &operator<<(std::ostream &stream, const Timing &timing);
private:
    std::string m_name;
    cudaEvent_t m_startEvent{};
    cudaEvent_t m_endEvent{};
    cudaStream_t m_cudaStream{};
    bool m_startEventSeen{false};
    bool m_endEventSeen{false};
    double m_elapsedSeconds{0};
    long int m_count{-5}; // start negative to allow warm up before counting events
};

// A collection of Timing events
class Timings
{
public:
    Timings();
    Timings(const Timings &other) = delete;
    Timings(Timings &&other) noexcept = delete;

    Timings &operator=(const Timings &other) = delete;
    Timings &operator=(Timings &&other) noexcept = delete;

    virtual ~Timings();

    void start(const std::string &name, cudaStream_t cudaStream);
    void end(const std::string &name);

    void endFrame(void);

    void setEnabled(const bool enabled);

    friend std::ostream &operator<<(std::ostream &stream, const Timings &timings);

private:
    std::map<std::string, Timing> m_timings{};
    bool m_enabled{true};
};

// A class for scoping a timing event - the event starts when the object is
// created and ends when the object is destroyed.
class ScopedEventTiming
{
public:
    ScopedEventTiming(Timings &timings, const std::string &name, cudaStream_t cudaStream);
    ScopedEventTiming(const ScopedEventTiming &other) = delete;
    ScopedEventTiming(ScopedEventTiming &&other) noexcept = delete;

    ScopedEventTiming &operator=(const ScopedEventTiming &other) = delete;
    ScopedEventTiming &operator=(ScopedEventTiming &&other) noexcept = delete;

    virtual ~ScopedEventTiming();

private:
    Timings &m_timings;
    std::string m_name;
};


#endif
