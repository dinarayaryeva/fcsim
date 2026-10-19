import select
import socket


class RecordingTransport:
    """Records outgoing data in memory (tests, virtual mode)."""

    def __init__(self):
        self.frames = []  # List of (t_us, data) pairs.

    def send(self, t_us, data):
        """Record a frame with its timestamp."""
        self.frames.append((t_us, data))

    def poll(self, timeout):
        """Wait for the specified timeout; no incoming data is available."""
        if timeout > 0:
            # Block without consuming CPU while waiting.
            select.select([], [], [], timeout)
        return []


class UdpTransport:
    """Send and receive frames over UDP."""

    def __init__(self, bind: str, peer: str = ""):
        # Parse the local address in "host:port" format.
        host, port = bind.rsplit(":", 1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, int(port)))

        # Non-blocking mode lets us drain available packets without waiting
        # for another packet to arrive.
        self.sock.setblocking(False)

        self.peer = None
        if peer:
            h, p = peer.rsplit(":", 1)
            self.peer = (h, int(p))

        # If no peer is configured, learn the destination from received data.
        self.learn_peer = not peer

    def send(self, t_us, data):
        """Send a frame to the configured or learned peer."""
        # Ignore outgoing frames until a peer is known.
        if self.peer:
            self.sock.sendto(data, self.peer)

    def poll(self, timeout):
        """Wait for incoming UDP data and return received payloads."""
        out = []

        # Wait until the socket is readable or the timeout expires.
        r, _, _ = select.select([self.sock], [], [], timeout)

        # Drain queued datagrams until the socket is empty.
        while r:
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                # No more datagrams are immediately available.
                break

            if self.learn_peer:
                # Use the sender as the destination for outgoing frames.
                self.peer = addr

            out.append(data)

        return out
