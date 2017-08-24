#include "pathUtil.h"
#include "tcpSocket.h"
#include "stringUtil.h"
#include <system_error>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

void TCPSocket::move(TCPSocket& sock)
{
    INetSocket::move(sock);

    _sockAddr  = sock._sockAddr;
    _mode      = sock._mode;
    sock._mode = eNone;
}

TCPSocket::TCPSocket(void)
    : INetSocket(ISocket::eTCP), _mode(eNone)
{
}

TCPSocket::TCPSocket(TCPSocket& sock)
    : INetSocket(ISocket::eTCP)
{
    move(sock);
}

TCPSocket& TCPSocket::operator= (TCPSocket& sock)
{
    move(sock);

    return *this;
}

TCPSocket::TCPSocket(TCPSocket&& sock) : INetSocket(ISocket::eTCP)
{
    move(sock);
}

TCPSocket::~TCPSocket(void)
{
    shutdown();
}

void TCPSocket::initialize(const string& hostAndPort) noexcept(false)
{
    TStringVec vec;

    StringUtil::split(vec, hostAndPort, ':');

    if( vec.size() != 2)
        throw std::runtime_error(string("Invalid host:port") + ERROR_LOCATION);

    initialize(vec[0], std::atoi(vec[1].c_str()));
}

void TCPSocket::initialize(const string& ip, uint16_t port) noexcept(false)
{
    _ifName = ip;
    _port   = port;
}

void TCPSocket::connect(void) noexcept(false)
{
    if( _ifName.empty() )
    {
        string  message("Please specify hostname");
        CLOSE_SOCKET( _fd[0] );
        throw std::runtime_error(message + ERROR_LOCATION);
    }

    setup();

    _mode = eClient;

    int result = ::connect( _fd[0], (SOCKADDR*) &_sockAddr,
                                    sizeof(_sockAddr) );

    //  switch (errno)  ECONNREFUSED
    if( result != INetSocket::SOCKET_OK )
    {
        _mode = eNone;

        ostringstream   message;
        message << hostAndPort();
        message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
        CLOSE_SOCKET( _fd[0] );
        throw std::runtime_error(message.str());
    }

    onStatusEvt.publish(hostAndPort(), ISocket::eConnected);
}

void TCPSocket::listen(uint8_t queueSize) noexcept(false)
{
    if( _fd[0] != ISocket::FD_INIT )
        throw std::runtime_error("Calling listen on open socket?" + ERROR_LOCATION);

    ostringstream message;

    setup();
    reuseAddr(TCPSocket::eEnable);
   //reusePort(TCPSocket::eEnable);

    _mode = eServer;

    int result = ::bind( _fd[0], (SOCKADDR*)&_sockAddr, sizeof(_sockAddr));

    if( result != INetSocket::SOCKET_OK )
    {
        _mode = eNone;

        message << "Failed To Bind Socket: ";
        if( _ifName.empty() )
            message << "localhost:" << _port;
        else
            message << _ifName << ":" << _port;

        message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;

        CLOSE_SOCKET( _fd[0] );
        throw std::runtime_error(message.str());
    }

    result = ::listen( _fd[0], queueSize);

    if( result != INetSocket::SOCKET_OK )
    {
        _mode = eNone;
        message << "Failed to listen on IFace";
        message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
        CLOSE_SOCKET( _fd[0] );
        throw std::runtime_error(message.str());
    }
}

TCPSocket TCPSocket::accept(void) noexcept(false)
{
    if( _fd[0] == ISocket::FD_INIT )
        throw std::runtime_error("Calling accept on closed socket?" + ERROR_LOCATION);

    struct sockaddr_storage clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);

    int32_t fd = ::accept( _fd[0], (SOCKADDR*)&clientAddr, &clientAddrSize);

    if( fd == ISocket::FD_INIT )
        throw std::runtime_error("Accept returned -1" + ERROR_LOCATION);

    onStatusEvt.publish(getIpPortStr(clientAddr), ISocket::eAccepted);

    if (clientAddr.ss_family == AF_INET) {
        // IPv4
        struct sockaddr_in *s = reinterpret_cast<struct sockaddr_in *>(&clientAddr);

        return TCPSocket(fd, getIpStr(clientAddr), static_cast<uint16_t>(ntohs(s->sin_port)));
    } else {
        // IPv6
        struct sockaddr_in6 *s = (struct sockaddr_in6 *)&clientAddr;

        return TCPSocket(fd, getIpStr(clientAddr), ntohs(s->sin6_port));
    }
}

TCPSocket* TCPSocket::acceptPtr(void)
{
    if( _fd[0] == ISocket::FD_INIT )
        return nullptr;

    struct sockaddr_storage clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);

    int32_t fd = ::accept( _fd[0], (SOCKADDR*)&clientAddr, &clientAddrSize);

    if( fd == ISocket::FD_INIT )
        return nullptr;

    onStatusEvt.publish(getIpPortStr(clientAddr), ISocket::eAccepted);

    if (clientAddr.ss_family == AF_INET) {
        // IPv4
        struct sockaddr_in *s = reinterpret_cast<struct sockaddr_in *>(&clientAddr);

        return new TCPSocket(fd, getIpStr(clientAddr), static_cast<uint16_t>(ntohs(s->sin_port)));
    } else {
        // IPv6
        struct sockaddr_in6 *s = (struct sockaddr_in6 *)&clientAddr;

        return new TCPSocket(fd, getIpStr(clientAddr), ntohs(s->sin6_port));
    }
}

void TCPSocket::shutdown(void)
{
    _spinning = false;
    _mode     = eNone;

    CLOSE_SOCKET( _fd[0] );

    if( _spinThread.joinable() )
    {
        _spinThread.join();

        onStatusEvt.publish(hostAndPort(), ISocket::eDisconnected);
    }
}

bool TCPSocket::pull(void)
{
    int32_t size = (this->*_reader)();

    if( size < 0 )
        return false;

    if( _storage._size < size)
    {
        if( _storage._buffer != nullptr)
            delete _storage._buffer;

        _storage._buffer = new int8_t[size];
        _storage._size   = size;
    }

    if( receive(_storage._buffer, size) < 0)
        return false;

    onDataEvt.publish(_storage._buffer, size);

    return true;
}

bool TCPSocket::spin(void)
{
    try
    {
        _spinThread = std::thread(&TCPSocket::threadClientHandler, this);
        return true;
    }
    catch(const std::system_error& ex) {
        onStatusEvt.publish( string("Failed to spawn tcp thread: ") + ex.what(),
                             ISocket::eDisconnected);
        return false;
    }
}

bool TCPSocket::spinServer(void)
{
    if(_ifName.empty() )
    {
        onStatusEvt.publish( string("Failed to spin tcp server: Missing host"),
                             ISocket::eError);
        return false;
    }

    if(_port == 0 )
    {
        onStatusEvt.publish( string("Failed to spin tcp server: Missing port"),
                             ISocket::eError);
        return false;
    }

    try
    {
        listen(1);
        _spinThread = std::thread(&TCPSocket::threadServerHandler, this);
        return true;
    }
    catch(const std::system_error& ex) {
        onStatusEvt.publish( string("Failed to spawn tcp thread: ") + ex.what(),
                             ISocket::eDisconnected);
        return false;
    }
}

void TCPSocket::reusePort(eState val)
{
    int32_t reusePort(0);

    if( val == eEnable )
        reusePort = 1;

    if( _fd[0] != ISocket::FD_INIT )
    {
//#if defined(SO_REUSEPORT)
        int ret = setsockopt(_fd[0], SOL_SOCKET, SO_REUSEPORT, (const char*)&reusePort, sizeof(reusePort) );

        if( ret == INetSocket::SOCKET_ERROR )
        {
            ostringstream message;

            message << "Failed to set SO_REUSEPORT on socket";
            message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
            CLOSE_SOCKET( _fd[0] );
            throw std::runtime_error(message.str());
        }
//#endif
    }
}

void TCPSocket::reuseAddr(eState val)
{
    int32_t reuseAddr(0);

    if( val == eEnable )
        reuseAddr = 1;

    if( _fd[0] != ISocket::FD_INIT )
    {
        int ret = setsockopt(_fd[0], SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddr, sizeof(reuseAddr) );

        if( ret == INetSocket::SOCKET_ERROR )
        {
            ostringstream  message;
            message << "Failed to set SO_REUSEADDR on socket";
            message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
            CLOSE_SOCKET( _fd[0] );
            throw std::runtime_error(message.str());
        }
    }
}

void TCPSocket::nagle(eState val)
{
    int32_t disableNagle(0);

    if( val == eDisable )
        disableNagle  = 1;     // Disable

    if( _fd[0] != ISocket::FD_INIT )
    {
        int ret = setsockopt(_fd[0], IPPROTO_TCP, TCP_NODELAY, (const char*)&disableNagle, sizeof(disableNagle) );

        if( ret == INetSocket::SOCKET_ERROR )
        {
            ostringstream  message;
            message << "Failed to set TCP_NODELAY on socket";
            message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
            CLOSE_SOCKET( _fd[0] );
            throw std::runtime_error( message.str() );
        }
    }
}

void TCPSocket::noneBlockingMode(eState val)
{
    int32_t disableNoneBlocking(0); // Enable Blocking

    if( val == eEnable )
        disableNoneBlocking = 1;  // Disable Blocking

    if( _fd[0] != ISocket::FD_INIT )
        ioctl(_fd[0], FIONBIO, &disableNoneBlocking);
}

int32_t TCPSocket::send( int8_t data ) const
{
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( uint8_t data ) const
{
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( int16_t data ) const
{
    data = HOST_TO_NET_S( data );
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( uint16_t data ) const
{
    data = HOST_TO_NET_S( data );
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( int32_t data ) const
{
    data = HOST_TO_NET_L( data );
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( uint32_t data ) const
{
    data = HOST_TO_NET_L( data );
    return send((const uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::send( const int8_t* pData, uint32_t size) const
{
    return send((const uint8_t*)pData, size);
}

int32_t TCPSocket::send( const uint8_t* pData, uint32_t size) const
{
    if( _fd[0] == ISocket::FD_INIT )
        return ISocket::FD_INIT;

    uint32_t sendCount(0);
    uint32_t totalSendCount(0);

    do
    {
        sendCount = ::send( _fd[0], pData + sendCount, size - sendCount, 0);

        if( sendCount <= 0)
        {
            if( errno == EAGAIN )
                continue;

            CLOSE_SOCKET( _fd[0] );
            return ISocket::FD_INIT;
        }

        totalSendCount += sendCount;

    } while( totalSendCount < size );

    return totalSendCount;
}

int32_t TCPSocket::receive( int8_t&   data ) const
{
    return receive( (uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::receive( uint8_t&  data ) const
{
    return receive( (uint8_t*)(&data), sizeof(data) );
}

int32_t TCPSocket::receive( int16_t&  data ) const
{
    int32_t ret = receive( (uint8_t*)(&data), sizeof(data) );

    if( ret > 0 )
        data = NET_TO_HOST_S( data );

    return ret;
}

int32_t TCPSocket::receive( uint16_t& data ) const
{
    int32_t ret = receive( (uint8_t*)(&data), sizeof(data) );

    if( ret > 0 )
        data = NET_TO_HOST_S( data );

    return ret;
}

int32_t TCPSocket::receive( int32_t&  data ) const
{
    int32_t ret = receive( (uint8_t*)(&data), sizeof(data) );

    if( ret > 0 )
        data = NET_TO_HOST_L( data );

    return ret;
}

int32_t TCPSocket::receive( uint32_t& data ) const
{
    int32_t ret = receive( (uint8_t*)(&data), sizeof(data) );

    if( ret > 0 )
        data = NET_TO_HOST_L( data );

    return ret;
}

int32_t TCPSocket::receive( int8_t* data, uint32_t size) const
{
    return receive((uint8_t*)data, size);
}

int32_t TCPSocket::receive( uint8_t*  data, uint32_t size) const
{
    if( _fd[0] == ISocket::FD_INIT )
        return ISocket::FD_INIT;

    int32_t idx  = 0;
    int32_t retValue;

    do
    {
        retValue = recv( _fd[0], (char*)(data+idx), (size - idx), 0 );

        if( retValue <= 0)
        {
            if( errno == EAGAIN )
                continue;

            CLOSE_SOCKET( _fd[0] );
            return retValue;
        }

        idx += retValue;

    } while( (uint32_t)idx < size );

    return idx;
}

std::ostream& operator <<(std::ostream &os, const TCPSocket& obj)
{
    os << "TCPSocket: " << obj._ifName << ":" << obj._port
       << " FD[" << obj._fd[0] << "]";

    return os;
}
