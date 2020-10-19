import socket


ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 3001
LEFT_TRACK_SPEED = -100
RIGHT_TRACK_SPEED = 100


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(bytes(f"{LEFT_TRACK_SPEED};{RIGHT_TRACK_SPEED}", "utf-8"),
                (ROBOT_IP, ROBOT_PORT))


if __name__ == '__main__':
    main()
