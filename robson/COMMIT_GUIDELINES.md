# Commit Standard - GitHub

## Introduction
This document describes the commit standard for this project on GitHub. Following these guidelines helps to maintain a clean, clear, and consistent commit history.

## Commit Message Structure
The commit message should follow the structure below:

1. **Type**: Indicates the type of change. Examples include: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`.
2. **Scope** (optional): Specific component of the code being changed. Example: `(mouth)`.
3. **Description**: Brief present-tense description of what was done. Example: `add lip sync code`.

**Format:**
<type>(<scope>): <description>

## Commit Types
Here are the most common commit types used in the project:

| Type      | Description                                                               |
|-----------|---------------------------------------------------------------------------|
| `feat`    | Adds a new feature.                                                       |
| `fix`     | Fixes a bug.                                                              |
| `docs`    | Changes to the documentation.                                             |
| `style`   | Changes that do not affect the meaning of the code (spaces, formatting, etc.). |
| `refactor`| Code changes that neither fix bugs nor add features.                      |
| `test`    | Adds or fixes tests.                                                      |
| `chore`   | Minor changes or adjustments, like updating dependencies.                |

## Example of a Well-Structured Commit Message
Here's an example of how a well-structured commit message should look:
feat(mouth): add lip sync based on audio

Adds a new lip sync feature to synchronize mouth movements with played audio. The synchronization is based on amplitude and frequency analysis of the audio.
 

## Best Practices
To keep the commit history clear and useful, follow these best practices:

1. Keep messages short and to the point (under 50 characters for the title).
2. Use present tense (e.g., 'fix bug', 'add feature').
3. Always include a more detailed description if necessary after a blank line.
4. Avoid very large commits; prefer to break them down into smaller parts.
Para o arquivo em português (GUIA_DE_COMMIT.md):
 
 
# Padrão de Commit - GitHub

## Introdução
Este documento descreve o padrão de commits para este projeto no GitHub. Seguir essas diretrizes ajuda a manter um histórico de commits limpo, claro e consistente.

## Estrutura da Mensagem de Commit
A mensagem de commit deve seguir a estrutura abaixo:

1. **Tipo**: Indica o tipo de mudança. Exemplos incluem: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`.
2. **Escopo** (opcional): Componente específico do código sendo alterado. Exemplo: `(mouth)`.
3. **Descrição**: Breve descrição no presente do que foi feito. Exemplo: `adiciona código de lip sync`.

**Formato:**
<tipo>(<escopo>): <descrição>

## Tipos de Commits
Aqui estão os tipos de commits mais comuns usados no projeto:

| Tipo      | Descrição                                                                 |
|-----------|---------------------------------------------------------------------------|
| `feat`    | Adiciona uma nova funcionalidade.                                         |
| `fix`     | Corrige um bug.                                                           |
| `docs`    | Alterações na documentação.                                               |
| `style`   | Mudanças que não afetam o significado do código (espaços, formatação, etc.). |
| `refactor`| Alteração de código que não corrige bugs nem adiciona funcionalidades.    |
| `test`    | Adiciona ou corrige testes.                                               |
| `chore`   | Mudanças menores ou ajustes, como atualizar dependências.                |

## Exemplo de Mensagem de Commit Bem Estruturada
Aqui está um exemplo de como deve ser uma mensagem de commit bem estruturada:
feat(mouth): adiciona sincronização labial com base no áudio

Adiciona uma nova funcionalidade de lip sync para sincronizar o movimento da boca com o áudio reproduzido. A sincronização é baseada na análise de amplitude e frequência do áudio.


## Boas Práticas
Para manter o histórico de commits claro e útil, siga estas boas práticas:

1. Mantenha mensagens curtas e objetivas (menos de 50 caracteres para o título).
2. Use o tempo verbal presente (ex.: 'corrige bug', 'adiciona funcionalidade').
3. Sempre inclua uma descrição mais detalhada se necessário após uma linha em branco.
4. Evite commits muito grandes; prefira dividir em partes menores. 