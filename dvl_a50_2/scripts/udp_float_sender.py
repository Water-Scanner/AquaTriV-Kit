#!/usr/bin/env python3

import socket
from typing import Callable, Optional


def send_float_ascii(ip: str, port: int, value: float, precision: int = 6) -> None:
	"""Send a single float immediately as ASCII/raw text to UDP ip:port.

	Args:
		ip: Destination IPv4 address, e.g. "192.168.2.1".
		port: Destination UDP port, e.g. 5000.
		value: Float value to send.
		precision: Decimal places in ASCII representation (default: 6).
	"""
	text = format(value, f".{precision}f")
	payload = text.encode("utf-8")
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	try:
		# Allow quick reuse on restart
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		sock.sendto(payload, (ip, port))
	finally:
		sock.close()


def create_float_ascii_sender(
	ip: str,
	port: int,
	precision: int = 6,
	broadcast: bool = False,
) -> Callable[[float], None]:
	"""Create a reusable sender function that sends floats as ASCII/raw to UDP.

	Usage:
		send = create_float_ascii_sender("192.168.2.1", 5000)
		send(3.14159)
		send(2.71828)
	
	Args:
		ip: Destination IPv4 address.
		port: Destination UDP port.
		precision: Decimal places in ASCII representation.
		broadcast: Enable UDP broadcast option on the socket.
	
	Returns:
		A function that accepts a float and sends it immediately.
	"""
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	# Allow quick reuse on restart
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	if broadcast:
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	destination = (ip, port)

	def _send(value: float) -> None:
		text = format(value, f".{precision}f")
		payload = text.encode("utf-8")
		sock.sendto(payload, destination)

	return _send


__all__ = [
	"send_float_ascii",
	"create_float_ascii_sender",
] 