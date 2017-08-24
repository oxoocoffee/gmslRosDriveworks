#ifndef SIGNAL_H
#define SIGNAL_H

// Author Robert J. Gebis

#include <functional>
#include <map>

// A signal object may call multiple slots with the
// same signature. You can connect functions to the signal
// which will be called when the publish() method on the
// signal object is invoked. Any argument passed to emit()
// will be passed to the given functions.

template <typename... Args>
class Signal
{
    typedef std::map<int, std::function<void(Args...)>> TSigMap;

    public:
    Signal() : _currentID(0)
    {
    }

    // copy creates new signal
    Signal(Signal const& other)
    {
        _currentID = other._currentID;
        _slots     = other._slots;
    }

    // connects a member function of a given object to this Signal
    template <typename F, typename... A>
    int connect(F&& f, A&& ... a) const
    {
        _slots.insert({++_currentID, std::bind(f, a...)});
        return _currentID;
    }

    template <class T>
    int connect(T* receiver, void(T::* f)(Args...) ) const
    {
        _slots.insert({++_currentID, [receiver, f](Args... args){ (receiver->*f)(args...); } });
        return _currentID;
    }

    // connects a std::function to the signal. The returned
    // value can be used to disconnect the function again
    int connect(std::function<void(Args...)> const& slot) const
    {
        _slots.insert(std::make_pair(++_currentID, slot));
        return _currentID;
    }

    // disconnects a previously connected function
    void disconnect(int id) const
    {
        _slots.erase(id);
    }

    // disconnects all previously connected functions
    void disconnect_all() const
    {
        _slots.clear();
    }

    // assignment creates new Signal
    Signal& operator=(Signal const& other)
    {
        disconnect_all();

        _slots = other._slots;
    }

    // calls all connected functions
    void publish(Args... p)
    {
        for(auto it : _slots)
            it.second(p...);
    }

    uint32_t count(void) const { return (uint32_t)_slots.size(); }

    private:
        mutable TSigMap _slots;
        mutable int32_t _currentID;
};

#endif // SIGNAL_H

