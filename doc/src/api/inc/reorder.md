The implementation of the stream ray query functions may re-order rays
arbitrarily and re-pack rays into ray packets of different size. For
this reason, callback functions may be invoked with an arbitrary
packet size (of size 1, 4, 8, or 16) and different ordering as specified
initially. For this reason, one may have to use the `rayID` component of
the ray to identify the original ray, e.g. to access a per-ray payload.
