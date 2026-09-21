package udpoutput

import (
	"net"
	"testing"
	"time"
)

func TestSend(t *testing.T) {
	server, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.IPv4zero, Port: 0})
	if err != nil {
		t.Fatal(err)
	}
	defer server.Close()

	sender, err := New(server.LocalAddr().String())
	if err != nil {
		t.Fatal(err)
	}
	defer sender.Close()

	expected := []byte{0xC8, 0x18, 0x16, 0x01, 0x02, 0x03}
	if err := sender.Send(expected); err != nil {
		t.Fatal(err)
	}

	buffer := make([]byte, 64)
	if err := server.SetReadDeadline(time.Now().Add(time.Second)); err != nil {
		t.Fatal(err)
	}
	n, _, err := server.ReadFromUDP(buffer)
	if err != nil {
		t.Fatal(err)
	}

	if string(buffer[:n]) != string(expected) {
		t.Fatalf("received % X, expected % X", buffer[:n], expected)
	}
}
