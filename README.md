# Guara

Código da simulação do **robô qudrupede Guará** que implementa o novo controle de andadura.
  - Construção do modelo computacional do robô.
  - Implementou-se um novo controle de andadura baseado no controle de trajetória das patas a partir dos torques nas juntas do Guará.
  - Implementada a classe GuaraLeg.
    - Possui atributos referentes as juntas da perna;
    - Implementa a função referente a cinemática inversa da pata;
  - Utilizou-se como parâmetros de otimização do controle a taxa de variação da quantidade de momento angular e linear no centro instantâneo de rotação do robô.
