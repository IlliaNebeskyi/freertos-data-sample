# freertos-data-sample
ESP-IDF FreeRTOS simple program to collect, process and output data


Value is read from pin A0 every 100ms using interrupts and stored in buffer;
First task calculates the average of read values each time the buffer fills and stores this average in a global variable.
Second task reads Serial input and whenever command "avg" is recieved - prints last calculated average.
