1. ODE hack
* Add diagonal term to A, which seems like Cfm;
* Add bouncing velocity to b
* fd from [-mu, mu]

2. Don't change Lemke simulation any more. Especially don't change the 0 to the
   user-defined zero. 

3. Tree-like structure skeleton will have contact points mainly of first few
   bodynodes, like bn0, bn0, bn0, bn0, bn1, bn2... After training, the
   accuracies are almost the same for different ct points. Therefore training
   LSTM is no longer needed.
