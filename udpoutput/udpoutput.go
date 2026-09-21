package udpoutput

import (
	"fmt"
	"net"
)

// Sender sends CRSF frames to a configured UDP endpoint.
type Sender struct {
	conn *net.UDPConn
}

// New creates a UDP sender for address, for example "127.0.0.1:9000".
func New(address string) (*Sender, error) {
	remote, err := net.ResolveUDPAddr("udp", address)
	if err != nil {
		return nil, fmt.Errorf("resolve UDP address %q: %w", address, err)
	}

	conn, err := net.DialUDP("udp", nil, remote)
	if err != nil {
		return nil, fmt.Errorf("open UDP connection to %q: %w", address, err)
	}

	return &Sender{conn: conn}, nil
}

// Send transmits one complete CRSF frame as one UDP datagram.
func (s *Sender) Send(frame []byte) error {
	n, err := s.conn.Write(frame)
	if err != nil {
		return fmt.Errorf("send UDP packet: %w", err)
	}
	if n != len(frame) {
		return fmt.Errorf("UDP sent %d bytes, expected %d", n, len(frame))
	}

	return nil
}

func (s *Sender) Close() error {
	return s.conn.Close()
}
