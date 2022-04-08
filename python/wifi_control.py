from time import time
from black import out
import numpy as np
import matplotlib.pyplot as plt
import telnetlib

host = "192.168.0.182"
port = 80
timeout = 100

esp01 = telnetlib.Telnet(host, port, timeout)

esp01.write(b"<Henlo Frenz>\r\n")

output = esp01.read_until(b"\r\n", 4)
print(output)