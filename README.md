��#   w e b s o c k e t 
 
 websockets is a library for building WebSocket servers and clients in Python with a focus on correctness, simplicity, robustness, and performance.

It supports several network I/O and control flow paradigms:

The default implementation builds upon asyncio, Python’s standard asynchronous I/O framework. It provides an elegant coroutine-based API. It’s ideal for servers that handle many clients concurrently.

The threading implementation is a good alternative for clients, especially if you aren’t familiar with asyncio. It may also be used for servers that don’t need to serve many clients.

The Sans-I/O implementation is designed for integrating in third-party libraries, typically application servers, in addition being used internally by websockets.
