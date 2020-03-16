 /*********************************************************************
 * dht11unimi.c : user space client to read temperature and humidity
 * 				  data from the driver dht11unimi_chrdev
 *********************************************************************/


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <openssl/ssl.h>
#include <openssl/err.h>

//#include <stdio.h>
//#include <sys/types.h>
//#include <ifaddrs.h>
//#include <netinet/in.h>
//#include <string.h>
//#include <arpa/inet.h>


#define TOKEN_RH			"RH: "
#define TOKEN_TEMPERATURE	"TEMP: "
#define HTTP_MESSAGE_SIZE 	2048
#define DRIVER_DATA_BUF 	1024
int sigint_received = 0;

int https_upload_to_host(char *hostname, int port, char *temp, char *rh, char *device_id);
int https_hello_server(char *hostname, int port);
int get_if_mac_address(char* src_if_name, char* dst_mac_address);
void ShowCerts(SSL* ssl);
SSL_CTX* InitCTX(void);

/*
 * Signal handler to quit the program :
 */
void sigint_handler(int signo) {
	sigint_received = 1;
	printf("\t\n***programm has received a SIGINT notification. End.***\t\n");
}

/*
 * Main program :
 */
int main(int argc,char **argv) {

	int readings = 0;
	char hostname[128] = "monitor-station-be.herokuapp.com";
	char mac_address[128] = {0};
	int port = 443;
	char driver_data[DRIVER_DATA_BUF];
	ssize_t nbytes;

	char *humidity_tok = NULL;
	char *temperature_tok = NULL;
	char *temperature = NULL;
	char *humidity = NULL;
	int ret;

	if (argc < 2) {
		printf("missing interface name. Stop.\n");
		exit(-1);
	}

	if (get_if_mac_address(argv[1], mac_address) < 0) {
		printf("Failed to retrieve mac address for interface %s. Impossible to upload data with unique device id\n", argv[1]);
		exit(-1);
	}
	printf("Device id mac address %s\n", mac_address);

	/* check the remote server is alive */
	printf("Contacting remote server %s\n", hostname);
	ret = https_hello_server(hostname, port);
	if (ret < 0) {
		printf("%s not available. Stop.\n", hostname);
		exit (-1);
	}
	printf("%s available to receive data\n", hostname);

/* TEST on X86 without driver */
//	sprintf(driver_data, "%s", "Sensor data RH: 37.4 TEMP: 19.2");
//	humidity_tok = strstr(driver_data, TOKEN_RH);
//	temperature_tok = strstr(driver_data, TOKEN_TEMPERATURE);
//	*temperature_tok = NULL;
//	humidity = humidity_tok + strlen(TOKEN_RH);
//	temperature = temperature_tok += strlen(TOKEN_TEMPERATURE);
//	ret = https_upload_to_host(hostname, port, temperature, humidity, mac_address);
//	if (ret < 0)
//		printf("Failed to upload data to %s\n", hostname);
//	else
//		printf("Device %s has successfully uploaded data temp %s rh %s\n", mac_address, temperature, humidity);
/* END TEST on X86 without driver */

	/* install sw-irq to stop the execution */
	signal(SIGINT, &sigint_handler);

	fd_set read_fds;

	/* open the driver */
	int fd = open("/dev/dht11unimi@0", O_RDONLY);
	if (fd < 0) {
		printf("failed to open dht11 driver (root permission required)\n");
		exit(fd);
	}

	/* main loop until a SIGINT is received */
	while (!sigint_received) {

		nbytes = 0;
		FD_ZERO(&read_fds);
		FD_SET(fd, &read_fds);

		printf("waiting data from the driver\n");
		select(FD_SETSIZE, &read_fds, NULL, NULL, NULL);

		if (FD_ISSET(fd, &read_fds))
			nbytes = read(fd, driver_data, sizeof(driver_data));

		if (nbytes > 0) {

			humidity_tok = strstr(driver_data, TOKEN_RH);
			if (humidity_tok != NULL) {
				++readings;
				driver_data[nbytes] = 0x00;
				/* Parse data from KERNEL */
				printf("dht11 driver says: %s\n", driver_data);
				temperature_tok = strstr(driver_data, TOKEN_TEMPERATURE);
				*temperature_tok = NULL;
				humidity = humidity_tok + strlen(TOKEN_RH);
				temperature = temperature_tok += strlen(TOKEN_TEMPERATURE);

				/*
				 * 		UPLOAD data to remote server
				 */
				ret = https_upload_to_host(hostname, port, temperature, humidity, mac_address);
				if (ret < 0)
					printf("Failed to upload data to %s\n", hostname);
				else
					printf("Device %s has successfully uploaded data temp %s rh %s\n", mac_address, temperature, humidity);
			} else {
				/* driver is telling us something without measurement's values */
				printf("Driver says %s\n", driver_data);
			}

		} else if (nbytes == 0)
			printf("nothing from the driver, waiting...\n");
		else
			printf("driver reading error, err. %ld\n", nbytes);

	    sleep(5);
	}

	close(fd);
	printf("Last Reading from the driver: %s, total readings %d\n", driver_data, readings);
	return 0;
}

int https_upload_to_host(char *hostname, int port, char *temp, char *rh, char *device_id) {

	struct sockaddr_in sa;		/* internet address structure */
	struct hostent *hp;			/* domain search result */
	char http_clientMessage[HTTP_MESSAGE_SIZE*2] = { 0 };
	char http_clientMessage_body[HTTP_MESSAGE_SIZE] = { 0 };
	char http_serverMessage[HTTP_MESSAGE_SIZE] = { 0 };
	ssize_t nbytes = -1;

	if ((hp = gethostbyname(hostname)) == NULL) {
		printf("Impossible to resolve remote hostname %s. Check your internet connection and dns configuration\n", hostname);
		return -1;
	}

	SSL_library_init();
	SSL_CTX *ctx = InitCTX();

	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("Failed to retrieve socket for network operations\n");
	    SSL_CTX_free(ctx);
		return -1;
	}

	memset(&sa,0,sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);

	memcpy(&sa.sin_addr.s_addr, hp->h_addr, hp->h_length);

	/* connect the socket */
	if (connect(sockfd,(struct sockaddr *)&sa, sizeof(sa)) < 0) {
		printf("Connect failed to host %s\n", hostname);
	    close(sockfd);
	    SSL_CTX_free(ctx);
	    return -1;
	}

    /*
     *		create http post method in order to upload data
     */

    //char http_message_body[1024] = "{ \"date\": \"\", \"humidityRel\": \"23.4\", \"macDeviceId\": \"77.77.77.77\", \"temperature\": \"12.7\"}";

    sprintf(http_clientMessage_body, "{ \"date\": \"\","
    						   	     " \"humidityRel\": \"%s\","
    						   	     " \"macDeviceId\": \"%s\","
    						   	     " \"temperature\": \"%s\"}",rh, device_id, temp);

    sprintf(http_clientMessage, "POST /dht11 HTTP/1.1\r\n"
		"Host: %s\r\n"
		"User-Agent: unimi/0.00.1\r\n"
		"accept: */*\r\n"
		"Content-Type: application/json\r\n"
		"Content-Length: %ld"
		"\r\n\r\n"
		"%s", hostname, strlen(http_clientMessage_body), http_clientMessage_body);

	/*
	 *			create https sockets over network socket and send data
	 */
	SSL *ssl = SSL_new(ctx);
    SSL_set_fd(ssl, sockfd);
    if (SSL_connect(ssl) <= 0) {
    	/* https failed */
        ERR_print_errors_fp(stderr);
    } else {
    	/* get any certs */
    	//ShowCerts(ssl);
        /* send message to the server */
    	nbytes = SSL_write(ssl, http_clientMessage, strlen(http_clientMessage));
        /*
         * socket are working in blocking mode
         * wait until the server sends a response or socket timeout will occurs
         */
    	nbytes = SSL_read(ssl, http_serverMessage, sizeof(http_serverMessage));
        http_serverMessage[nbytes] = 0;
        //printf("Received: \"%s\"\n", http_serverMessage);
        /* release data allocated for the connection */
        SSL_free(ssl);
    }

    /* close network socket and release ssl context in memory */
    close(sockfd);
    SSL_CTX_free(ctx);

    return nbytes;
}


/*
 *		Retrieve echo hello message from the remote server
 *		This function help in two way.
 *		1 - Tell us that network and server are ready
 *		2 - Wakeup from hibernation the herokuapp virtual VM
 *		    This is the way to have free VM on internet. Sorry
 *
 */
int https_hello_server(char *hostname, int port) {

	struct sockaddr_in sa;		/* internet address structure */
	struct hostent *hp;			/* domain search result */
	char acClientRequest[1024] = {0};
	char buf[HTTP_MESSAGE_SIZE];
	ssize_t nbytes = -1;

	if ((hp = gethostbyname(hostname)) == NULL) {
		printf("Impossible to resolve remote hostname %s. Check your internet connection and dns configuration\n", hostname);
		free(hostname);
		return -1;
	}

	SSL_library_init();
	SSL_CTX *ctx = InitCTX();

	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("Failed to retrieve socket for network operations\n");
		return -1;
	}

	memset(&sa,0,sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);

	memcpy(&sa.sin_addr.s_addr, hp->h_addr, hp->h_length);

	/* connect the socket */
	if (connect(sockfd,(struct sockaddr *)&sa, sizeof(sa)) < 0) {
		printf("Connect failed to host %s.|n", hostname);
		return -1;
	}

    /*
     *		simple http get method
     */
        sprintf(acClientRequest, "GET https://www.%s/echo HTTP/1.1\r\n"
        		"Host: %s\r\n"
        		"User-Agent: curl/7.65.3\r\n"
        		"accept: */*\r\n"
        		"\r\n\r\n", hostname, hostname);

	/*
	 *			create https sockets over network socket and send data
	 */
	SSL *ssl = SSL_new(ctx);
    SSL_set_fd(ssl, sockfd);
    if (SSL_connect(ssl) <= 0) {
    	/* https failed */
        ERR_print_errors_fp(stderr);
    } else {
    	/* get any certs */
    	//ShowCerts(ssl);
        /* send message to the server */
    	nbytes = SSL_write(ssl, acClientRequest, strlen(acClientRequest));   /* encrypt & send message */
        /* socket are working in blocking mode
         * wait until the server sends a response or socket timeout will occurs
         */
    	nbytes = SSL_read(ssl, buf, sizeof(buf)); /* get reply & decrypt */
        buf[nbytes] = 0;
        //printf("Received: \"%s\"\n", buf);
        /* release data allocated for the connection */
        SSL_free(ssl);
    }

    /* close network socket and release ssl context in memory */
    close(sockfd);
    SSL_CTX_free(ctx);

    if ((nbytes > 0) && (strstr(buf, "hello") == NULL)) {
    	printf("failed to retrieve echo hello from the server. Remote machine isn't still ready. Retry later");
    	return -1;
    }

    return nbytes;
}

int get_if_mac_address(char* src_if_name, char* dst_mac_address) {

	struct ifreq s;				/* network interface data */
	char *p = dst_mac_address;

	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("Failed to retrieve socket for network operations. Stop.");
		return sockfd;
	}

	/*
	 *		Retrieve device mac address
	 */
	strcpy(s.ifr_name, src_if_name);
	if (ioctl(sockfd, SIOCGIFHWADDR, &s) != 0) {
		close(sockfd);
	    return -1;
	}

	for (int i = 0; i < 6; ++i)
		if (i == 0)
			p += sprintf(p, "%02x", (unsigned char) s.ifr_addr.sa_data[i]);
		else
			p += sprintf(p, ":%02x", (unsigned char) s.ifr_addr.sa_data[i]);

	close(sockfd);
	return 0;

}

SSL_CTX* InitCTX(void) {
	SSL_METHOD *method;
    SSL_CTX *ctx;
    OpenSSL_add_all_algorithms();  /* Load cryptos, et.al. */
    SSL_load_error_strings();   /* Bring in and register error messages */
    method = TLSv1_2_client_method();  /* Create new client-method instance */
    ctx = SSL_CTX_new(method);   /* Create new context */
    if (ctx == NULL) {
        ERR_print_errors_fp(stderr);
        abort();
    }
    return ctx;
}

/*
 *		Support functino to show which https certificare from Server
 *		are in use
 */
void ShowCerts(SSL* ssl) {
    X509 *cert;
    char *line;
    cert = SSL_get_peer_certificate(ssl); /* get the server's certificate */
    if (cert != NULL) {
        printf("Server certificates:\n");
        line = X509_NAME_oneline(X509_get_subject_name(cert), 0, 0);
        printf("Subject: %s\n", line);
        free(line);       /* free the malloc'ed string */
        line = X509_NAME_oneline(X509_get_issuer_name(cert), 0, 0);
        printf("Issuer: %s\n", line);
        free(line);       /* free the malloc'ed string */
        X509_free(cert);     /* free the malloc'ed certificate copy */
    } else
        printf("Info: No client certificates configured.\n");
}
