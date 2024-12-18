#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#define MAX_DADOS 7        // Tamanho máximo do campo de dados
#define QUEUE_SIZE 10      // Número máximo de pacotes na fila
#define MSG_SIZE 7     // Tamanho máximo do buffer que o callback do input utiliza
#define STACK_SIZE 1024
#define PRIORITY 5 

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define GPIOB_NODE DT_NODELABEL(gpiob)
#define TX_PIN 3  // Pino de saída
#define RX_PIN 2  // Pino de entrada

const struct device *gpio_dev;

typedef struct {
    uint8_t u;                 // Preâmbulo
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

struct k_mutex canal_mutex;
struct k_condvar condvar;

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

void csma_thread(void *arg1, void *arg2, void *arg3) {
    if (!gpio_dev) {
        printk("Erro ao inicializar GPIO_DEV\n");
        return;
    }

    /* Configura o pino RX como entrada */
    gpio_pin_configure(gpio_dev, RX_PIN, GPIO_INPUT);

    int max = 20;
    int incremento = 50;
    int tempo_aleatorio;

    while (1) {
        k_mutex_lock(&canal_mutex, K_FOREVER);

        /* Verifica 32 bits e converte para 8 bits */
        int canal_ocupado = 0; // Indica se algum bit 1 foi encontrado
        for (int i = 0; i < 8; i++) {
            int bit = ler_canal_filtrado(gpio_dev, RX_PIN);
            if (bit == 1) {
                canal_ocupado = 1;
            }
        }

        if (canal_ocupado) {
            /* Se o canal estiver ocupado, espera um tempo aleatório crescente */
            tempo_aleatorio = rand() % max;
            max += incremento;
            printk("Canal ocupado, aguardando %d ms antes de verificar novamente...\n", tempo_aleatorio);
            k_mutex_unlock(&canal_mutex);
            k_sleep(K_MSEC(tempo_aleatorio));
        } else {
            /* Se o canal estiver livre, sinaliza e reinicia o tempo aleatório */
            printk("Canal livre, notificando transmissora.\n");
            max = 20;
            k_condvar_signal(&condvar);
            k_mutex_unlock(&canal_mutex);
            k_sleep(K_MSEC(10));
        }
    }
}

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

void transmitter_thread(void) {
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
    gpio_dev = DEVICE_DT_GET(GPIOB_NODE);

    if (!device_is_ready(gpio_dev)) {
        printk("Erro: GPIOB não está pronto!\n");
        return;
    }

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

K_THREAD_DEFINE(transmitter_tid, STACK_SIZE, transmitter_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
