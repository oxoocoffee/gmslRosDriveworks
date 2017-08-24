#ifndef I_NET_SOCKET_H
#define I_NET_SOCKET_H

// Author Robert J. Gebis

#include "iSocket.h"
#include "pathUtil.h"
#include <arpa/inet.h>
#include <cstring>

class INetSocket : public ISocket
{
    protected:
        typedef int32_t (INetSocket::*Reader)(void);

        INetSocket(int32_t sockFD, eType type, uint32_t size = 4)
            : ISocket(type), _port(0), _msgHdrSize(size)
        { _fd[0] = sockFD; setMsgHdrSize(size); }

        struct Buffer
        {
            int8_t*  _buffer;
            int32_t  _size;

            Buffer(void) : _buffer(nullptr), _size(0) {}
        };

    public:
        static const int32_t    SOCKET_OK    = 0;
        static const int32_t    SOCKET_ERROR = -1;

        typedef struct sockaddr SOCKADDR;

        INetSocket(eType type, uint32_t size = 4)
            : ISocket(type), _port(0), _msgHdrSize(size)
        { setMsgHdrSize(size); }

        const string hostAndPort(void) const
        {
            return _ifName + ":" + std::to_string(_port);
        }

        virtual int32_t send( int8_t            data ) const = 0;
        virtual int32_t send( uint8_t           data ) const = 0;
        virtual int32_t send( int16_t           data ) const = 0;
        virtual int32_t send( uint16_t          data ) const = 0;
        virtual int32_t send( int32_t           data ) const = 0;
        virtual int32_t send( uint32_t          data ) const = 0;
        virtual int32_t send( const int8_t*     pData, uint32_t size) const = 0;
        virtual int32_t send( const uint8_t*    pData, uint32_t size) const = 0;

        virtual int32_t receive( int8_t&        data ) const = 0;
        virtual int32_t receive( uint8_t&       data ) const = 0;
        virtual int32_t receive( int16_t&       data ) const = 0;
        virtual int32_t receive( uint16_t&      data ) const = 0;
        virtual int32_t receive( int32_t&       data ) const = 0;
        virtual int32_t receive( uint32_t&      data ) const = 0;
        virtual int32_t receive( int8_t*        data, uint32_t size) const = 0;
        virtual int32_t receive( uint8_t*       data, uint32_t size) const = 0;

        void    setMsgHdrSize(uint32_t size)
        {
            _msgHdrSize = size;

            if(_msgHdrSize == 1)
                _reader = &INetSocket::readByte;
            else if(_msgHdrSize == 2)
                _reader = &INetSocket::readWord;
            else if(_msgHdrSize == 4)
                _reader = &INetSocket::readDWord;
            else {
#if DEBUG
                throw std::runtime_error("Possible values must be 1, 2 or 4" +
                                         ERROR_LOCATION);
#endif
            }
        }

        typedef Signal<const int8_t*, uint32_t> TMessageEvent;

        TMessageEvent   onDataEvt;

    protected:
        void    move(INetSocket& sock)
        {
            ISocket::move(sock);

            _storage     = sock._storage;
            _port        = sock._port;

            sock._port            = 0;
            sock._storage._buffer = nullptr;
            sock._storage._size   = 0;

            setMsgHdrSize(sock._msgHdrSize);
        }

        //Convert a struct sockaddr address to a string, IPv4 and IPv6:

        string getIpStr(const struct sockaddr_storage& sa)
        {
            char str[INET6_ADDRSTRLEN] = {0};

            switch(sa.ss_family)
            {
                case AF_INET:
                    inet_ntop(AF_INET, &(( (struct sockaddr_in *)(&sa) )->sin_addr), str, INET6_ADDRSTRLEN);
                    break;

                case AF_INET6:
                    inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)(&sa))->sin6_addr), str, INET6_ADDRSTRLEN);
                    break;

                default:
                    strncpy(str, "Unknown AF", INET6_ADDRSTRLEN);
                    return NULL;
            }

            return string(str);
        }

        // Get "host:port"
        string getIpPortStr(const struct sockaddr_storage& sa)
        {
            char   str[INET6_ADDRSTRLEN] = {0};
            string port;

            switch(sa.ss_family) {
                case AF_INET:
                    inet_ntop(AF_INET, &(( (struct sockaddr_in *)(&sa) )->sin_addr), str, INET6_ADDRSTRLEN);
                    port = ntohs(((struct sockaddr_in *)(&sa))->sin_port);
                    break;

                case AF_INET6:
                    inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)(&sa))->sin6_addr), str, INET6_ADDRSTRLEN);
                    port = ntohs(((struct sockaddr_in6 *)(&sa))->sin6_port);
                    break;

                default:
                    return string("Unknown AF");
            }

            return string(str) + ":" + port;
        }

        virtual int32_t readByte(void)  = 0;
        virtual int32_t readWord(void)  = 0;
        virtual int32_t readDWord(void) = 0;

    protected:
        uint16_t            _port;
        Reader              _reader;
        struct sockaddr_in	_sockAddr;
        uint32_t            _msgHdrSize;   // number of bytes used to designate message size;
        Buffer              _storage;
};

#if ENABLE_HOST2NET
    #define     NET_TO_HOST_S(a)    ntohs(a)
    #define     NET_TO_HOST_L(a)    ntohl(a)
    #define     HOST_TO_NET_S(a)    htons(a)
    #define     HOST_TO_NET_L(a)    htonl(a)
#else
    #define     NET_TO_HOST_S(a)    a
    #define     NET_TO_HOST_L(a)    a
    #define     HOST_TO_NET_S(a)    a
    #define     HOST_TO_NET_L(a)    a
#endif // ENABLE_HOST2NET

#endif // I_NET_SOCKET_H
