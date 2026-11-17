# Mathematical model

The one-way link requests `q12=max(0,k12(h1-h2))`. In normal operation,
`A1*h1_dot=u-q12` and `A2*h2_dot=q12-k2*h2-d`. Adding these equations cancels
the internal transfer and proves the total-volume balance. The simulator uses
start-of-step states for every requested flow and controller value.
