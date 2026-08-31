// abs(double) overload shim for the official demo (legacy abs(double) no longer resolves).
inline double abs(double x) { return x < 0.0 ? -x : x; }
