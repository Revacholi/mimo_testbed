#include <arpa/inet.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define PORT 8080
#define BUFFER_SIZE 8192

int main() {
  int server_fd, client_fd;
  struct sockaddr_in server_addr, client_addr;
  socklen_t client_len = sizeof(client_addr);
  char buffer[BUFFER_SIZE];

  // Create socket
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    perror("Socket creation failed");
    return 1;
  }

  // Define server address
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(PORT);

  // Bind socket to port
  if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
    perror("Bind failed");
    return 1;
  }

  // Listen for connections
  if (listen(server_fd, 1) < 0) {
    perror("Listen failed");
    return 1;
  }

  printf("Server listening on port %d...\n", PORT);

  // Accept client connection
  if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                          &client_len)) < 0) {
    perror("Accept failed");
    return 1;
  }

  printf("Client connected. Awaiting ...\n");

  bool transmission = false;
  while (1) {
    memset(buffer, 0, BUFFER_SIZE);
    ssize_t bytes_read = recv(client_fd, buffer, BUFFER_SIZE, 0);
    if (bytes_read > 0) {
      printf("Received: %s\n", buffer);
      if (strcmp(buffer, "transmit") == 0) {
        transmission = true;
        snprintf(buffer, BUFFER_SIZE, "Starting transmission");
        if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
          perror("Send failed");
          break;
        }
        printf("Starting transmission...\n");
        // Upload data to ram and start loop transmission
      } else if (strcmp(buffer, "stop") == 0) {
        snprintf(buffer, BUFFER_SIZE, "Stopped transmission");
        if (transmission == false) {
          perror("Not transmitting");
          break;
        } else if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
          perror("Send failed");
          break;
        }
        printf("Transmission stopped\n");
        transmission = false;
        // Stop the transmission loop
      } else if (strcmp(buffer, "recieve") == 0) {
        snprintf(buffer, BUFFER_SIZE, "Starting recieving");
        if (transmission == true) {
          perror("Transmitting");
          break;
        } else if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
          perror("Send failed");
          break;
        }
        printf("Starting recieving\n");
      } else {
        snprintf(buffer, BUFFER_SIZE, "Unkown command");
        if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
          perror("Send failed");
          break;
        }
        perror("Unkown command\n");
      }
    } else if (bytes_read == 0) {
      printf("Client disconnected.\n");
      break;
    } else {
      perror("Receive failed");
      break;
    }
  }
  close(client_fd);
  close(server_fd);
  return 0;
}
