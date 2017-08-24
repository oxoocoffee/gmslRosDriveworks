#include "pathUtil.h"
#include "tcpSocket.h"
#include <arpa/inet.h>
#include <netdb.h>

// This is used to wrap BSD socket
TCPSocket::TCPSocket(int32_t sockFD, eMode mode)
 : INetSocket(sockFD, ISocket::eTCP), _mode(mode)
{
}

TCPSocket::TCPSocket(int32_t  sockFD, string peer,
                     uint16_t port)
 : INetSocket(sockFD, ISocket::eTCP), _mode(eClient)
{
    _ifName = peer;
    _port   = port;
}

void TCPSocket::threadClientHandler(void)
{
    onStatusEvt.publish("TCPSocket spawned",
                        ISocket::eInfo);


    uint32_t errorCounter(0);

    while(_spinning)
    {
        try
        {
            connect();
            errorCounter = 0;
        }
        catch(const std::runtime_error& ex)
        {
            if( !( errorCounter % 10 ) )
                onStatusEvt.publish(ex.what(), ISocket::eDisconnected);

            ++errorCounter;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        while(_spinning)
        {
            if( pull() == false )
            {
                if(_spinning)
                {
                    onStatusEvt.publish("TCPSocket read error",
                                        ISocket::eDisconnected);

                    _mode = eNone;
                    CLOSE_SOCKET( _fd[0] );

                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }

                break;
            }
        };
    }

    onStatusEvt.publish("TCPSocket quiting",
                        ISocket::eInfo);
}

void TCPSocket::threadServerHandler(void)
{
    onStatusEvt.publish(hostAndPort(), ISocket::eListening);

    while(_spinning)
    {
        TCPSocket* tcp = acceptPtr();

        if( tcp == nullptr or _spinning == false)
            break;

        string host = tcp->hostAndPort();

        onStatusEvt.publish(host, ISocket::eConnected);

        // publish is blocking call
        // tcp pointer is deleted by handler
        onConnectionEvt.publish(tcp);

        // WARNING. tcp pointer is dead at this point

        onStatusEvt.publish(host, ISocket::eDisconnected);
    }
}

void TCPSocket::setup(void) noexcept(false)
{
    if( _fd[0] != ISocket::FD_INIT )
        return;

    _fd[0] = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if( _fd[0] == ISocket::FD_INIT )
    {
        ostringstream message;
        message << "Failed to create TCP Socket";
        message << ", Reason: " << std::strerror(errno) << ERROR_LOCATION;
        throw std::runtime_error(message.str());
    }

    memset( (void*)&_sockAddr, 0, sizeof(_sockAddr) );

    string ifStr(_ifName);

    transform(ifStr.begin(), ifStr.end(), ifStr.begin(), ::toupper);

    if( _ifName.empty() || (ifStr == "ANY") )
        _sockAddr.sin_addr.s_addr   = htonl( INADDR_ANY );
    else
    {
        _sockAddr.sin_addr.s_addr   = inet_addr( _ifName.c_str() );

        if( _sockAddr.sin_addr.s_addr == 0xffffffff)
        {
            // Not a dotted-notation IP address a.b.c.d
            struct hostent* hptr = (struct hostent *) gethostbyname( _ifName.c_str() );

            if( hptr == 0L )
            {
                string  message("Failed to resolve hostname. Host: ");
                        message += _ifName;
                        message += ", Reason: ";
                        message += std::strerror(errno);
                        message += ERROR_LOCATION;
                CLOSE_SOCKET( _fd[0] );
                throw std::runtime_error(message);
            }

            memcpy( &_sockAddr.sin_addr, *((struct in_addr **)hptr->h_addr_list), sizeof(struct in_addr));
        }
    }

    _sockAddr.sin_family = AF_INET;
    _sockAddr.sin_port   = htons( _port );
}

int32_t TCPSocket::readByte(void)
{
    uint8_t val;

    if( receive(val) < 0)
        return -1;

    return val;
}

int32_t TCPSocket::readWord(void)
{
    uint16_t val;

    if( receive(val) < 0)
        return -1;

    return val;
}

int32_t TCPSocket::readDWord(void)
{
    uint32_t val;

    if( receive(val) < 0)
        return -1;

    return val;
}
