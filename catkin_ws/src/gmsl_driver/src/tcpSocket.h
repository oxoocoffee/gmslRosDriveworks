#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

// Author Robert J. Gebis

#include "iNetSocket.h"
#include <stdexcept>
#include <thread>

class TCPSocket : public INetSocket
{
    public:
        typedef std::unique_ptr<TCPSocket>  TUniTCPPtr;
        typedef Signal<TCPSocket*>          TConnectionEvent;

        enum eMode
        {
            eNone = 0,
            eClient,
            eServer
        };

        enum eState
        {
            eDisable = 0,
            eEnable
        };

         TCPSocket(void);

         // Moves socket. NOT COPY
         TCPSocket(TCPSocket& sock);

         // Moves socket. NOT COPY
         TCPSocket& operator= (TCPSocket& sock);

         // Moves socket. NOT COPY
         TCPSocket(TCPSocket&& sock);

        ~TCPSocket(void) override;

         /// TCP initialization
         /// hostAndPort - host:port
        void       initialize(const string& hostAndPort) noexcept(false);

        /// TCP initialization
        /// host - ip or host name
        /// port - port
        void       initialize(const string& host,
                             uint16_t      port) noexcept(false);

        void       connect(void) noexcept(false) override;

        void       listen(uint8_t queueSize = 5) noexcept(false);
        TCPSocket  accept(void)  noexcept(false);

        // Returned needs to be managed byt caller.
        TCPSocket* acceptPtr(void);

        void       shutdown(void)    override;

        eMode      mode(void) const { return _mode; }
        bool       pull(void)        override;

        // Spin new thread and connect to host
        bool       spin(void)        override;
        // Spin new thread and listen for new clients
        bool       spinServer(void);

        void   reusePort(eState val);
        void   reuseAddr(eState val);
        void   nagle(eState val);
        void   noneBlockingMode(eState val);  // eEnable - enable none blocking

        int32_t send( int8_t            data ) const override;
        int32_t send( uint8_t           data ) const override;
        int32_t send( int16_t           data ) const override;
        int32_t send( uint16_t          data ) const override;
        int32_t send( int32_t           data ) const override;
        int32_t send( uint32_t          data ) const override;
        int32_t send( const int8_t*     pData, uint32_t size) const override;
        int32_t send( const uint8_t*    pData, uint32_t size) const override;

        int32_t receive( int8_t&        data ) const override;
        int32_t receive( uint8_t&       data ) const override;
        int32_t receive( int16_t&       data ) const override;
        int32_t receive( uint16_t&      data ) const override;
        int32_t receive( int32_t&       data ) const override;
        int32_t receive( uint32_t&      data ) const override;
        int32_t receive( int8_t*        data, uint32_t size) const override;
        int32_t receive( uint8_t*       data, uint32_t size) const override;

        TConnectionEvent    onConnectionEvt;

    private:
        void    threadClientHandler(void);
        void    threadServerHandler(void);
        void    setup(void) noexcept(false);
        void    move(TCPSocket& sock);

        int32_t readByte(void)  override;
        int32_t readWord(void)  override;
        int32_t readDWord(void) override;

    private:
        std::thread         _spinThread;
        eMode               _mode;

        TCPSocket(int32_t sockFD, eMode mode);
        TCPSocket(int32_t sockFD, string peer, uint16_t port);

        friend std::ostream& operator<<(std::ostream &os, const TCPSocket& obj);
};

std::ostream& operator <<(std::ostream &os, const TCPSocket& obj);

#endif // TCP_SOCKET_H

