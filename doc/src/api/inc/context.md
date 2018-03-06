The intersection context (`context` argument) can specify flags to
optimize traversal and a filter callback function to be invoked for
every intersection. Further, the pointer to the intersection context is
propagated to callback functions invoked during traversal and can thus
be used to extend the ray with additional data. See Section
`RTCIntersectContext` for more information.
