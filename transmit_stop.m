function [] = transmit_stop(client)
write(client, "stop");
while (client.NumBytesAvailable == 0)
    pause(0.1);
end
response = read(client, client.NumBytesAvailable, 'uint8');
disp(char(response));
end
