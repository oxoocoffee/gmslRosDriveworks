#ifndef I_SOCKET_H
#define I_SOCKET_H

// Author Robert J. Gebis

#include "sigSlot.h"
#include <memory>
#include <sys/select.h>
#include <netinet/in.h>
#include <unistd.h>

class ISocket
{
    public:
        enum eType
        {
            eTCP,
            eUDP,
        };

        enum eStatus
        {
            eInfo,
            eWarn,
            eError,
            eDisconnected,
            eConnected,
            eAccepted,
            eListening
        };

        typedef Signal<const string, const eStatus>   TStatusEvent;

        typedef std::unique_ptr<ISocket>              TSocket;

        static constexpr const char*    ANY_DEV      = "any";
        static constexpr const int32_t  FD_INIT      = -1;

                 ISocket(eType type) : _type(type) {}

        virtual ~ISocket(void) {}

        inline eType    type(void)  const { return _type; }
        inline string   ifName(void)const { return _ifName; }
        inline bool     isConnected(void) const { return _fd[0] != ISocket::FD_INIT; }

        virtual void    connect(void) noexcept(false) = 0;
        virtual void    shutdown(void) = 0;
        virtual bool    pull(void)     = 0;
        virtual bool    spin(void)     = 0;

        virtual void    interrupSpin(void) { _spinning = false; }

        TStatusEvent    onStatusEvt;

    protected:
        void    move(ISocket& sock)
        {
            _ifName   = std::move(sock._ifName);
            _fd[0]    = sock._fd[0];
            _rdfd     = sock._rdfd;
            _timeout  = sock._timeout;

            sock._fd[0] = ISocket::FD_INIT;
        }

    protected:
        eType           _type;
        string          _ifName;
        bool            _spinning = true;
        mutable int32_t _fd[1]    = { FD_INIT };
        fd_set          _rdfd;
        timeval         _timeout  = { 1, 0 };
};

#define CLOSE_SOCKET(s)                     \
{                                           \
    if( s != ISocket::FD_INIT )             \
    {                                       \
        ::shutdown(s, SHUT_RDWR);           \
        ::close(s);                         \
        s = ISocket::FD_INIT;               \
    }                                       \
}

#endif // I_SOCKET_H

