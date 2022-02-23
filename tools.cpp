  startMillis = millis();  //initial start time
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  ....
    
    
int number = intQueue.dequeue();    // Will return number 1 and remove it from the queue
int number = intQueue.getHead();    // Will return number 123 but leave it still in the queue
int number = intQueue.dequeue();    // Will return number 123 and remove it from the queue
intQueue.enqueue(1);    // Adds number 1 to the queue
