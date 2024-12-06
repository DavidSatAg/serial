#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#define MAX_DADOS 7        // Tamanho máximo do campo de dados
#define QUEUE_SIZE 10      // Número máximo de pacotes na fila
#define MSG_SIZE            /// Tamanho máximo do buffer que o callback do input utiliza
#define STACK_SIZE 1024
#define PRIORITY 5 

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

typedef struct {
    uint8_t u;                  // Preâmbulo
    uint8_t sync;              // SYNC (1 byte)
    uint8_t stx;               // STX (1 byte)
    uint8_t id_n;              // ID e N combinados (1 byte)
    uint8_t dados[MAX_DADOS];  // DADOS (até 7 bytes)
    uint8_t etx;               // ETX (1 byte)
} Pacote;

K_MSGQ_DEFINE(uart_msgq, sizeof(Pacote), QUEUE_SIZE, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

void encapsular_pacote(Pacote *pacote, const char *dados, uint8_t id) {
    uint8_t tamanho_dados = strlen(dados) > MAX_DADOS ? MAX_DADOS : strlen(dados);

    pacote->u = 0x55; // Preâmbulo
    pacote->sync = 0xAA; // Valor de SYNC
    pacote->stx = 0x02;  // Valor de STX
    pacote->id_n = (id << 3) | (tamanho_dados & 0x07); // Combinação de ID e N
    memset(pacote->dados, 0, MAX_DADOS); // Zera os dados
    memcpy(pacote->dados, dados, tamanho_dados); // Copia os dados
    pacote->etx = 0x03; // Valor de ETX
}

void serial_cb(const struct device *dev, void *user_data)
{
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
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* Preencher o pacote */
            Pacote pacote;
            encapsular_pacote(&pacote, rx_buf, id);

            /* Enviar para a fila */
            if (k_msgq_put(&uart_msgq, &pacote, K_NO_WAIT) != 0) {
                printk("Fila cheia! Pacote descartado.\n");
            } else {
                printk("Pacote enviado para a message queue: ID=%d, Tamanho=%d\n", id, rx_buf_pos);
            }

            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

void consumer_thread(void) {
    Pacote pacote_consumido; // Variável para armazenar o pacote retirado da fila

    while (1) {
        // Tenta obter um pacote da fila (bloqueia até que haja um pacote disponível)
        if (k_msgq_get(&uart_msgq, &pacote_consumido, K_FOREVER) == 0) {
            // Imprime todos os campos do pacote
            printk("Pacote recebido:\n");
            printk("u: %d\n", pacote_consumido.u);
            printk("SYNC: 0x%02X\n", pacote_consumido.sync);
            printk("STX: 0x%02X\n", pacote_consumido.stx);
            printk("ID_N: 0x%02X\n", pacote_consumido.id_n);
            printk("DADOS: ");
            for (int i = 0; i < MAX_DADOS; i++) {
                printk("0x%02X ", pacote_consumido.dados[i]);
            }
            printk("\n");
            printk("ETX: 0x%02X\n", pacote_consumido.etx);
        } else {
            printk("Erro ao obter pacote da fila.\n");
        }

        k_msleep(10);
    }
}


void main(void)
{
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return;
    }

    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    printk("UART Echo Bot iniciado\n");

    while (1) {
        k_msleep(1000);
    }
}

K_THREAD_DEFINE(consumer_tid, STACK_SIZE, consumer_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
