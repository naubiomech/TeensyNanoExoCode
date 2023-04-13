# Adding New UART Messages

## Messaging Structure
Each message has a get and update ID. The get is used to request data from the other microcontroller, the update is used to set data recieved from the other microcontroller. Naturally, a get is followed by an update.
messages will have a get and an update ID, but every message ID must be unique. 

Each message ID has a message handler, this is the code that is ran on the microcontroller when it recieves a message from the other microncontroller. 

## Declare Message ID(s)
In the src folder, go to uart_commands.h. Find the namespace 'UART_command_names' and declare your message ID(s). 

## Declare Message Handler
In the same file (uart_commands.h), find the namespace 'UART_command_handlers' and create your function. The function must 
have the following function signature:
    static inline void your_funtion_name(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)

## Add your function to the list of callable functions
In the same file (uart_commands.h), find the namespace 'UART_command_utils', within that namespace there is a function named
'handle_msg'. In handle_msg, add a switch case(s) for your message. 

## Done
Now you can test your new function!