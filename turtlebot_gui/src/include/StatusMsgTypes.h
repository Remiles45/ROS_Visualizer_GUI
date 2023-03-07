#pragma once
/*
Defines the types of possible statuses that the topic could have
*/

enum msg_type {
    Connected,
    Disconnected,
    Unresponsive,
    TryingToConnect
};