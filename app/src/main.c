#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

#define MAX_DADOS 7        // Máximo de dados no campo do pacote
#define QUEUE_SIZE 10      // Máximo de pacotes na fila
#define MSG_SIZE 7         // Tamanho do buffer para entrada da UART
#define STACK_SIZE 1024
#define PRIORITY 5 

#define TX_INTERVAL K_MSEC(10)

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define GPIOB_NODE DT_NODELABEL(gpiob)
#define TX_PIN 3  // Pino de saída (transmissão)
#define RX_PIN 2  // Pino de entrada (recepção)

// Pacote
typedef struct {
    uint8_t u;                 // Preâmbulo (esperado: 0x55)
    uint8_t sync;              // SYNC (esperado: 0xAA)
    uint8_t stx;               // STX (esperado: 0x02)
    uint8_t id_n;              // Combinação de ID e N (N = número de dados, nos 3 bits da direita)
    uint8_t dados[MAX_DADOS];  // Dados (até 7 bytes)
    uint8_t etx;               // ETX (esperado: 0x03)
} Pacote;

// Message Queue
K_MSGQ_DEFINE(uart_msgq, sizeof(Pacote), QUEUE_SIZE, 4);
K_MSGQ_DEFINE(rx_msgq, sizeof(Pacote), QUEUE_SIZE, 4);

// Mapeando os pinos usando o device tree
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *gpio_dev;

// Buffer para pegar o input do usuário
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

// Mutex e condvar para o CSMA atuar junto com a thread que transmite
K_MUTEX_DEFINE(canal_mutex);
K_CONDVAR_DEFINE(condvar);

/* (Opcional) Mutex para acesso ao pino RX se necessário para sincronização entre CSMA e recepção */
K_MUTEX_DEFINE(rx_mutex);

// Variáveis para a transmissão
static Pacote current_tx_packet;
static int tx_byte_index = 0;
static int tx_bit_index = 0;
static bool tx_in_progress = false;
static const int total_bytes = sizeof(Pacote);

static struct k_timer tx_timer;

// Função para ler o valor do pino de recepção, lê 4 vezes mais rápido que a escrita pra evitar ruído.
int ler_canal_filtrado(const struct device *gpio_dev, int rx_pin) {
    int contador_1 = 0;
    int contador_0 = 0;
    for (int i = 0; i < 4; i++) {
        int estado = gpio_pin_get(gpio_dev, rx_pin);
        if (estado == 1) {
            contador_1++;
        } else {
            contador_0++;
        }
        k_sleep(K_USEC(2500));
    }
    return (contador_1 > contador_0) ? 1 : 0;
}

// Estados que a thread de recepção pode estar
typedef enum {
    WAIT_FOR_PREAMBLE,
    WAIT_FOR_SYNC,
    WAIT_FOR_STX,
    WAIT_FOR_ID_N,
    WAIT_FOR_DATA,
    WAIT_FOR_ETX
} rx_state_t;

// Variáveis para a recepção
static rx_state_t rx_state = WAIT_FOR_PREAMBLE;
static Pacote rx_packet;
static int data_expected = 0;
static int data_index = 0;

// Processamento e verificação se o byte é válido
void process_received_byte(uint8_t byte) {
    switch (rx_state) {
    case WAIT_FOR_PREAMBLE:
        if (byte == 0x55) {
            rx_packet.u = byte;
            rx_state = WAIT_FOR_SYNC;
        }
        break;
    case WAIT_FOR_SYNC:
        if (byte == 0xAA) {
            rx_packet.sync = byte;
            rx_state = WAIT_FOR_STX;
        } else {
            rx_state = WAIT_FOR_PREAMBLE;
        }
        break;
    case WAIT_FOR_STX:
        if (byte == 0x02) {
            rx_packet.stx = byte;
            rx_state = WAIT_FOR_ID_N;
        } else {
            rx_state = WAIT_FOR_PREAMBLE;
        }
        break;
    case WAIT_FOR_ID_N:
        rx_packet.id_n = byte;
        data_expected = byte & 0x07;
        if (data_expected > MAX_DADOS) {
            rx_state = WAIT_FOR_PREAMBLE;
        } else if (data_expected == 0) {
            rx_state = WAIT_FOR_ETX;
        } else {
            data_index = 0;
            rx_state = WAIT_FOR_DATA;
        }
        break;
    case WAIT_FOR_DATA:
        rx_packet.dados[data_index++] = byte;
        if (data_index >= data_expected) {
            rx_state = WAIT_FOR_ETX;
        }
        break;
    case WAIT_FOR_ETX:
        if (byte == 0x03) {
            rx_packet.etx = byte;
            if (k_msgq_put(&rx_msgq, &rx_packet, K_NO_WAIT) != 0) {
                printk("Receptor: Fila tá cheia, pacote descartado.\n");
            } else {
                printk("Receptor: Pacote recebido.\n");
            }
        }
        rx_state = WAIT_FOR_PREAMBLE;
        break;
    }
}

// Thread de recepção
void receiver_thread(void *arg1, void *arg2, void *arg3) {
    uint8_t rx_current_byte = 0;
    int rx_bit_count = 0;
    while (1) {
        int bit = ler_canal_filtrado(gpio_dev, RX_PIN);
        rx_current_byte = (rx_current_byte << 1) | (bit & 0x01);
        rx_bit_count++;
        if (rx_bit_count >= 8) {
            process_received_byte(rx_current_byte);
            rx_bit_count = 0;
            rx_current_byte = 0;
        }
    }
}

// Thread de impressão
void rx_print_thread(void *arg1, void *arg2, void *arg3) {
    Pacote pacote;
    while (1) {
        if (k_msgq_get(&rx_msgq, &pacote, K_FOREVER) == 0) {
            int n = pacote.id_n & 0x07;
            printk("RX Packet: ID_N: 0x%02X, Dados: ", pacote.id_n);
            for (int i = 0; i < n; i++) {
                printk("0x%02X ", pacote.dados[i]);
            }
            printk("\n");
        }
    }
}

// Timer responsável por transmitir os bits
void tx_timer_handler(struct k_timer *timer) {
    if (tx_byte_index < total_bytes) {
        uint8_t current_byte = ((uint8_t *)&current_tx_packet)[tx_byte_index];
        int bit_to_send = (current_byte >> (7 - tx_bit_index)) & 1;
        gpio_pin_set(gpio_dev, TX_PIN, bit_to_send);
        printk("TX Timer: Sent bit %d (byte %d, bit %d)\n", bit_to_send, tx_byte_index, tx_bit_index);
        tx_bit_index++;
        if (tx_bit_index >= 8) {
            tx_bit_index = 0;
            tx_byte_index++;
        }
    } else {
        k_timer_stop(&tx_timer);
        tx_in_progress = false;
        printk("TX Timer: Transmissão completa.\n");
    }
}

// Thread transmissora
void transmitter_thread(void *arg1, void *arg2, void *arg3) {
    Pacote pacote;
    while (1) {
        k_mutex_lock(&canal_mutex, K_FOREVER);
        k_condvar_wait(&condvar, &canal_mutex, K_FOREVER);
        k_mutex_unlock(&canal_mutex);

        if (k_msgq_get(&uart_msgq, &pacote, K_FOREVER) == 0) {
            memcpy(&current_tx_packet, &pacote, sizeof(Pacote));
            tx_byte_index = 0;
            tx_bit_index = 0;
            tx_in_progress = true;
            k_timer_start(&tx_timer, TX_INTERVAL, TX_INTERVAL);
            while (tx_in_progress) {
                k_sleep(K_MSEC(10));
            }
        } else {
            printk("Transmissor: Não deu pra pegar o pacote.\n");
        }
    }
}

// Thread do CSMA
void csma_thread(void *arg1, void *arg2, void *arg3) {
    if (!gpio_dev) {
        printk("Erro ao inicializar GPIO_DEV\n");
        return;
    }

    gpio_pin_configure(gpio_dev, RX_PIN, GPIO_INPUT);

    int max = 20;
    int incremento = 50;
    int tempo_aleatorio;

    while (1) {
        k_mutex_lock(&canal_mutex, K_FOREVER);

        int canal_ocupado = 0;
        for (int i = 0; i < 8; i++) {
            int bit = ler_canal_filtrado(gpio_dev, RX_PIN);
            if (bit == 1) {
                canal_ocupado = 1;
            }
        }

        if (canal_ocupado) {
            tempo_aleatorio = rand() % max;
            max += incremento;
            printk("CSMA: Canal ocupado, aguardando %d ms antes de verificar novamente...\n", tempo_aleatorio);
            k_mutex_unlock(&canal_mutex);
            k_sleep(K_MSEC(tempo_aleatorio));
        } else {
            printk("CSMA: Canal livre, notificando transmissora.\n");
            max = 20;
            k_condvar_signal(&condvar);
            k_mutex_unlock(&canal_mutex);
            k_sleep(K_MSEC(10));
        }
    }
}

// Callback para pegar o input do usuário, semelhante ao exemplo do echo_bot
void serial_cb(const struct device *dev, void *user_data) {
    static uint8_t id = 29; // ID do pacote
    uint8_t c;
    if (!uart_irq_update(uart_dev)) {
        return;
    }
    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            rx_buf[rx_buf_pos] = '\0';
            Pacote pacote;
            uint8_t tamanho = strlen(rx_buf) > MAX_DADOS ? MAX_DADOS : strlen(rx_buf);
            pacote.u = 0x55;
            pacote.sync = 0xAA;
            pacote.stx = 0x02;
            pacote.id_n = (id << 3) | (tamanho & 0x07);
            memset(pacote.dados, 0, MAX_DADOS);
            memcpy(pacote.dados, rx_buf, tamanho);
            pacote.etx = 0x03;
            if (k_msgq_put(&uart_msgq, &pacote, K_NO_WAIT) != 0) {
                printk("SerialCB: Fila cheia! Pacote descartado.\n");
            } else {
                printk("SerialCB: Pacote na fila: ID=%d, Tamanho=%d\n", id, rx_buf_pos);
            }
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

void main(void) {
    gpio_dev = DEVICE_DT_GET(GPIOB_NODE);
    if (!device_is_ready(gpio_dev)) {
        printk("Main: Error - GPIOB not ready!\n");
        return;
    }
    if (!device_is_ready(uart_dev)) {
        printk("Main: UART device not ready!\n");
        return;
    }
    if (gpio_pin_configure(gpio_dev, TX_PIN, GPIO_OUTPUT) < 0) {
        printk("Main: Error configuring TX_PIN\n");
        return;
    }
    if (gpio_pin_configure(gpio_dev, RX_PIN, GPIO_INPUT) < 0) {
        printk("Main: Error configuring RX_PIN\n");
        return;
    }
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    k_timer_init(&tx_timer, tx_timer_handler, NULL);

    printk("System started.\n");
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

// Criação das threads
K_THREAD_DEFINE(transmitter_tid, STACK_SIZE, transmitter_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(csma_tid, STACK_SIZE, csma_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(receiver_tid, STACK_SIZE, receiver_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(rx_print_tid, STACK_SIZE, rx_print_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
