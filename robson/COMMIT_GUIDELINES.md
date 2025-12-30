# Padrão de Commit – Projeto de Robótica Móvel (Webots)

## Introdução

Este documento define o padrão de commits adotado neste projeto de robótica móvel, desenvolvido no simulador Webots, utilizando o robô Pioneer 3-DX equipado com sensor LiDAR, com foco em mapeamento do ambiente e cálculo de trajetória em tempo real.

A adoção de um padrão de commits claro e consistente tem como objetivo:
- Facilitar a manutenção e a evolução do código;
- Tornar o histórico de desenvolvimento legível e rastreável;
- Apoiar a organização de experimentos e ajustes de parâmetros;
- Atender boas práticas de desenvolvimento de software e pesquisa científica.

Este padrão é inspirado em *Conventional Commits*, com adaptações específicas para projetos de robótica e simulação.

---

## Estrutura da Mensagem de Commit

Toda mensagem de commit deve seguir o formato abaixo:

```text
<tipo>(<escopo>): <descrição>
```

### Componentes da mensagem

- **Tipo**: indica a natureza da mudança realizada.
- **Escopo** (opcional): indica o subsistema ou módulo afetado.
- **Descrição**: frase curta, no tempo presente, descrevendo objetivamente a mudança.

### Exemplo

```text
feat(mapping): add occupancy grid generation
```

---

## Tipos de Commit

| Tipo | Descrição |
|------|----------|
| feat | Adiciona uma nova funcionalidade |
| fix | Corrige um bug |
| docs | Alterações na documentação |
| style | Ajustes de formatação |
| refactor | Refatoração sem mudança de funcionalidade |
| test | Adição ou correção de testes |
| chore | Ajustes gerais ou manutenção |
| perf | Melhoria de desempenho |
| param | Ajuste de parâmetros |
| exp | Experimentos científicos |
| sim | Alterações na simulação |
| data | Dados, mapas ou logs |

---

## Escopos Padrão

| Escopo | Descrição |
|-------|-----------|
| sim | Simulação no Webots |
| robot | Modelo do Pioneer 3-DX |
| lidar | Aquisição e processamento do LiDAR |
| mapping | Mapeamento do ambiente |
| localization | Localização e estimação de pose |
| planning | Planejamento de trajetória |
| control | Controle de movimento |
| avoidance | Desvio de obstáculos |
| nav | Navegação integrada |
| config | Configurações |
| utils | Funções auxiliares |
| docs | Documentação |
| exp | Experimentos |

---

## Exemplos de Commits

```text
feat(lidar): implement scan preprocessing
feat(mapping): generate occupancy grid map
fix(control): correct angular velocity saturation
param(planning): tune A* heuristic weight
perf(nav): reduce trajectory computation time
exp(mapping): evaluate map accuracy under lidar noise
docs(exp): document mapping experiment results
```

---

## Boas Práticas

1. Mantenha commits pequenos, objetivos e focados.
2. Utilize sempre o tempo verbal presente.
3. Separe commits de código, ajustes de parâmetros e experimentos.
4. Mudanças que impactam resultados experimentais devem ser commitadas separadamente.
5. Evite commits grandes e genéricos.
6. O histórico deve permitir compreender e reproduzir a evolução do projeto.

---

## Considerações Finais

Este padrão busca equilibrar boas práticas de engenharia de software com as necessidades específicas de projetos de robótica e pesquisa acadêmica. Um histórico de commits bem estruturado contribui diretamente para a organização, reprodutibilidade e qualidade do projeto.
