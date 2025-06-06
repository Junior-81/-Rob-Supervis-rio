# Robô Simulado com Controle Supervisório

## Descrição do Projeto

Este projeto apresenta a simulação de um robô móvel (AGV – Veículo Guiado Automaticamente) controlado remotamente por um painel supervisório. A simulação ocorre em ambiente 3D com PyBullet, enquanto a comunicação entre painel e robô utiliza o protocolo MQTT. O robô responde a comandos básicos (“frente”, “ré”, “esquerda”, “direita” e “parar”) e detecta obstáculos, emitindo alertas sempre que algo estiver próximo.

---

## Objetivos

- Simular um robô móvel em ambiente 3D com obstáculos aleatórios.
- Permitir controle remoto via painel supervisório.
- Enviar a posição do robô em tempo real para o painel.
- Detectar obstáculos automaticamente e emitir alertas.
- Garantir que alertas não bloqueiem o funcionamento do robô.

---

## Tecnologias Utilizadas

- **Python**: Linguagem principal do sistema.
- **PyBullet**: Simulação física do ambiente e objetos.
- **MQTT (Mosquitto Broker)**: Comunicação em tempo real entre robô e painel.
- **Node-RED**: Painel de controle interativo via navegador.

---

## Abordagem e Implementação

### Robô Simulado (Python + PyBullet)
- Robô modelado em ambiente 3D com obstáculos posicionados aleatoriamente.
- Recebe comandos via MQTT e move os motores conforme solicitado.
- Publica continuamente sua posição para o painel.
- Detecta obstáculos próximos usando PyBullet e envia alertas sem interromper a operação.

### Painel de Controle (Node-RED)
- Interface web intuitiva com botões de controle (frente, ré, parar, esquerda, direita).
- Envia comandos MQTT ao robô e exibe sua posição em tempo real.
- Apresenta alertas de obstáculos detectados.
- Acesso remoto facilitado via navegador.

---

## Conclusão

O projeto integra simulação física e controle remoto em tempo real de forma robusta. Todos os objetivos foram atingidos, desde a movimentação remota até a recepção de alertas ambientais. A solução é escalável para múltiplos robôs, sensores e fontes de dados, demonstrando na prática um sistema de controle supervisório com MQTT e visualização em tempo real.

---

## Links do Projeto

- [YouTube] https://www.youtube.com/watch?v=RNUIgsZoNa0