# serial

Fala, professor, estamos tentando entender como o echo_bot funciona.

Vamos tentar fazer a transmissão da seguinte forma:
- Uma thread para guardar o que o usuário está digitando e contar o número de bytes da mensagem
- Uma thread para enviar o SYNC, STX, ID, N, DADOS, ETX

Como no exemplo do echo_bot ele trabalha com o K_MSGQ_DEFINE(), estamos no processo de ler a documentação e entender o Message Queues.
Se esse caminho não for bom envie um e-mail ou a gente conversa na quinta.
