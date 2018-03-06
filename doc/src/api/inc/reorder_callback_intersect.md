When performing ray queries using `rtcIntersect1`, it is guaranteed
that the packet size is 1 when the callback is invoked. When
performing ray queries using the `rtcIntersect4/8/16` functions, it is
not generally guaranteed that the ray packet size (and order of rays
inside the packet) passed to the callback matches the initial ray
packet. However, under some circumstances these properties are
guaranteed, and whether this is the case can be queried using
`rtcGetDeviceProperty`. When performing ray queries using the stream
API such as `rtcIntersect1M`, `rtcIntersect1Mp`, `rtcIntersectNM`, or
`rtcIntersectNp` the order of rays and ray packet size of the callback
function might change to either 1, 4, 8, or 16.

For many usage scenarios, repacking and re-ordering of rays does not
cause difficulties in implementing the callback function. However,
algorithms that need to extend the ray with additional data must use
the `rayID` component of the ray to identify the original ray to access
the per-ray data.
