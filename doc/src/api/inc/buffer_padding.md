When the buffer will be used as a vertex buffer (`RTC_BUFFER_TYPE_VERTEX`
and `RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE`), the last buffer element must be
readable using 16-byte SSE load instructions, thus padding the last
element is required for certain layouts. E.g. a standard `float3` vertex
buffer layout should add storage for at least one more float to the
end of the buffer.
