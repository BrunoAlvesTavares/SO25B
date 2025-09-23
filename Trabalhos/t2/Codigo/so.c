// so.c
// sistema operacional
// simulador de computador
// so25b

// ---------------------------------------------------------------------
// INCLUDES {{{1
// ---------------------------------------------------------------------

#include "so.h"
#include "dispositivos.h"
#include "err.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"

#include <stdlib.h>
#include <stdbool.h>


// ---------------------------------------------------------------------
// CONSTANTES E TIPOS {{{1
// ---------------------------------------------------------------------

// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas
#define MAX_PROC 16

typedef enum { P_MORTO=0, P_PRONTO=1, P_EXEC=2 } p_estado_t;

typedef struct {
  int pid;
  p_estado_t estado;
  // registradores salvos
  int A, X, PC, ERRO;
  // terminal "base" para E/S (A/B/C/D) em forma de enum de dispositivos
  int term_base; // D_TERM_A / D_TERM_B / D_TERM_C / D_TERM_D
  bool em_uso;
} pcb_t;

static int pid_next = 1;


static int escolhe_term_base_por_pid(int pid) {
  int s = (pid-1) % 4;
  switch (s) {
    case 0: return D_TERM_A;
    case 1: return D_TERM_B;
    case 2: return D_TERM_C;
    default: return D_TERM_D;
  }
}

static int dev_teclado_ok(int base)   { return base + TERM_TECLADO_OK; }
static int dev_teclado(int base)      { return base + TERM_TECLADO; }
static int dev_tela_ok(int base)      { return base + TERM_TELA_OK; }
static int dev_tela(int base)         { return base + TERM_TELA; }

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  int regA, regX, regPC, regERRO; // cópia do estado da CPU
  // t2: tabela de processos, processo corrente, pendências, etc
    // ---- Parte I: tabela de processos e escalonador simples
  pcb_t proc[MAX_PROC];
  int   idx_atual;   // índice na tabela do processo em execução, -1 se nenhum
  int   n_procs;     // quantidade de entradas em uso
};


// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);


// ---------------------------------------------------------------------
// CRIAÇÃO {{{1
// ---------------------------------------------------------------------

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // ---- init T2: tabela de processos
  self->idx_atual = -1;
  self->n_procs = 0;
  for (int i = 0; i < MAX_PROC; i++) {
    self->proc[i].em_uso = false;
    self->proc[i].estado = P_MORTO;
    self->proc[i].pid = 0;
    self->proc[i].A = 0;
    self->proc[i].X = 0;
    self->proc[i].PC = 0;
    self->proc[i].ERRO = 0;
    self->proc[i].term_base = D_TERM_A; // valor default; será definido ao criar
  }

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  free(self);
}


// ---------------------------------------------------------------------
// TRATAMENTO DE INTERRUPÇÃO {{{1
// ---------------------------------------------------------------------

// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
//   interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
//   a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
//   da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupção em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupção
static int so_trata_interrupcao(void *argC, int reg_A)
{
  so_t *self = argC;
  irq_t irq = reg_A;
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupção
  so_trata_irq(self, irq);
  // faz o processamento independente da interrupção
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  so_escalona(self);
  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{
  // Lê registradores salvos pelo tratador em memória
  int A, PC, ERRO, X;
  if (mem_le(self->mem, CPU_END_A, &A) != ERR_OK
   || mem_le(self->mem, CPU_END_PC, &PC) != ERR_OK
   || mem_le(self->mem, CPU_END_erro, &ERRO) != ERR_OK
   || mem_le(self->mem, 59, &X) != ERR_OK) {
    console_printf("SO: erro na leitura dos registradores");
    self->erro_interno = true;
    return;
  }

  // Espelha no "snapshot" do SO (usado para id da syscall, mensagens de erro, etc.)
  self->regA = A;
  self->regX = X;
  self->regPC = PC;
  self->regERRO = ERRO;

  // Se houver processo corrente, salva também no PCB
  if (self->idx_atual >= 0) {
    pcb_t *p = &self->proc[self->idx_atual];
    p->A = A; 
    p->X = X; 
    p->PC = PC; 
    p->ERRO = ERRO;
  }
}

static void so_trata_pendencias(so_t *self)
{
  // t2: realiza ações que não são diretamente ligadas com a interrupção que
  //   está sendo atendida:
  // - E/S pendente
  // - desbloqueio de processos
  // - contabilidades
  // - etc
}

static void so_escalona(so_t *self)
{
  // Escalonador simples:
  // - se há processo atual e ele está PRONTO/EXEC, continua (preferência)
  // - senão, escolhe o primeiro PRONTO da tabela

  // Normaliza estado do atual: ao entrar no SO por interrupção,
  // consideramos que ele fica PRONTO (a menos que tenha morrido)
  if (self->idx_atual >= 0) {
    if (self->proc[self->idx_atual].estado == P_EXEC)
      self->proc[self->idx_atual].estado = P_PRONTO;
  }

  // mantém se ainda pode executar
  if (self->idx_atual >= 0) {
    pcb_t *p = &self->proc[self->idx_atual];
    if (p->em_uso && p->estado == P_PRONTO) {
      p->estado = P_EXEC;
      return;
    }
  }

  // procura primeiro PRONTO
  for (int i = 0; i < MAX_PROC; i++) {
    if (self->proc[i].em_uso && self->proc[i].estado == P_PRONTO) {
      self->idx_atual = i;
      self->proc[i].estado = P_EXEC;
      return;
    }
  }

  // nenhum pronto -> sem processo CPU ficará ociosa
  self->idx_atual = -1;
}

static int so_despacha(so_t *self)
{
  // Copia do PCB escolhido para a “área de retorno” que a RETI vai restaurar
  if (self->idx_atual < 0) {
    // Sem processo: deixa PC parado no tratador para “girar” (ou poderia saltar p/ BIOS)
    // Aqui mantemos o estado que já estava.
    return 0; // valor de retorno de CHAMAC (não usamos)
  }

  pcb_t *p = &self->proc[self->idx_atual];

  if (mem_escreve(self->mem, CPU_END_A,     p->A)    != ERR_OK
   || mem_escreve(self->mem, CPU_END_PC,    p->PC)   != ERR_OK
   || mem_escreve(self->mem, CPU_END_erro,  p->ERRO) != ERR_OK
   || mem_escreve(self->mem, 59,            p->X)    != ERR_OK) {
    console_printf("SO: erro na escrita dos registradores");
    self->erro_interno = true;
  }

  // Ao voltar com RETI, a CPU voltará em modo usuário com PC do processo.
  return 0;
}


// ---------------------------------------------------------------------
// TRATAMENTO DE UMA IRQ {{{1
// ---------------------------------------------------------------------

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupção que está acontecendo, e atende de acordo
  switch (irq) {
    case IRQ_RESET:
      so_trata_reset(self);
      break;
    case IRQ_SISTEMA:
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      so_trata_irq_relogio(self);
      break;
    default:
      so_trata_irq_desconhecida(self, irq);
  }
}

// chamada uma única vez, quando a CPU inicializa
static void so_trata_reset(so_t *self)
{
  // coloca o tratador de interrupção na memória
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != CPU_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  // --- Parte T2: criar processo init (PID 1) e preparar a tabela de processos
  // limpa/normaliza a tabela
  for (int i = 0; i < MAX_PROC; i++) {
    self->proc[i].em_uso = false;
    self->proc[i].estado = P_MORTO;
    self->proc[i].pid = 0;
    self->proc[i].A = 0;
    self->proc[i].X = 0;
    self->proc[i].PC = 0;
    self->proc[i].ERRO = 0;
    self->proc[i].term_base = D_TERM_A;
  }
  self->n_procs = 0;
  self->idx_atual = -1;
  pid_next = 1;

  // carrega o init.maq
  ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  // cria PCB para o init
  int slot = 0;
  pcb_t *p = &self->proc[slot];
  p->em_uso   = true;
  p->pid      = pid_next++;
  p->estado   = P_PRONTO;
  p->A = 0; 
  p->X = 0; 
  p->ERRO = 0;
  p->PC       = ender;              // ponto de entrada do init
  p->term_base = escolhe_term_base_por_pid(p->pid);
  self->n_procs = 1;

  // Deixa o escalonador escolher (vai pegar o init agora).
  // Não escrevemos registradores da CPU diretamente; o despachante cuidará
  // de carregar o contexto do processo escolhido quando o SO retornar.
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  // Ocorreu um erro interno na CPU
  // O erro está codificado em CPU_END_erro
  // Em geral, causa a morte do processo que causou o erro
  // Ainda não temos processos, causa a parada da CPU
  // t2: com suporte a processos, deveria pegar o valor do registrador erro
  //   no descritor do processo corrente, e reagir de acordo com esse erro
  //   (em geral, matando o processo)
  err_t err = self->regERRO;
  console_printf("SO: IRQ não tratada -- erro na CPU: %s", err_nome(err));
  self->erro_interno = true;
}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // rearma o interruptor do relógio e reinicializa o timer para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }
  // t2: deveria tratar a interrupção
  //   por exemplo, decrementa o quantum do processo corrente, quando se tem
  //   um escalonador com quantum
  console_printf("SO: interrupção do relógio (não tratada)");
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}


// ---------------------------------------------------------------------
// CHAMADAS DE SISTEMA {{{1
// ---------------------------------------------------------------------

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  // t2: com processos, o reg A deve estar no descritor do processo corrente
  int id_chamada = self->regA;
  console_printf("SO: chamada de sistema %d", id_chamada);
  switch (id_chamada) {
    case SO_LE:
      so_chamada_le(self);
      break;
    case SO_ESCR:
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);
      // t2: deveria matar o processo
      self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  // Parte I: ainda com espera ocupada (como no esqueleto),
  // porém lendo do terminal associado ao processo corrente.
  if (self->idx_atual < 0) { self->regA = -1; return; }
  pcb_t *p = &self->proc[self->idx_atual];
  int base = p->term_base;

  for (;;) {
    int ok;
    if (es_le(self->es, dev_teclado_ok(base), &ok) != ERR_OK) {
      console_printf("SO: problema no acesso ao estado do teclado");
      self->erro_interno = true; return;
    }
    if (ok) break;
    console_tictac(self->console);     // como não está saindo do SO, a unidade de controle não está executando seu laço.
    // esta gambiarra faz pelo menos a console ser atualizada
    // t2: com a implementação de bloqueio de processo, esta gambiarra não
    //   deve mais existir.
  }

  int ch;
  if (es_le(self->es, dev_teclado(base), &ch) != ERR_OK) {
    console_printf("SO: problema no acesso ao teclado");
    self->erro_interno = true; return;
  }

  p->A = ch;   // resultado da chamada vai em A do processo
  self->regA = 0; // retorno da syscall (OK)
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
  if (self->idx_atual < 0) { self->regA = -1; return; }
  pcb_t *p = &self->proc[self->idx_atual];
  int base = p->term_base;
  int ch = p->X; // caractere a escrever vem no X do processo

  for (;;) {
    int ok;
    if (es_le(self->es, dev_tela_ok(base), &ok) != ERR_OK) {
      console_printf("SO: problema no acesso ao estado da tela");
      self->erro_interno = true; return;
    }
    if (ok) break;
    console_tictac(self->console);
  }

  if (es_escreve(self->es, dev_tela(base), ch) != ERR_OK) {
    console_printf("SO: problema na escrita na tela");
    self->erro_interno = true; return;
  }

  self->regA = 0; // OK
}

// implementação da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // precisa ter um processo em execução (o chamador)
  if (self->idx_atual < 0) { 
    self->regA = -1; 
    return; 
  }

  pcb_t *pai = &self->proc[self->idx_atual];

  // lê a string com o nome do executável a partir do endereço em X do chamador
  char nome[128];
  if (!copia_str_da_mem(sizeof(nome), nome, self->mem, pai->X)) {
    // endereço inválido, sem '\0' no limite, etc.
    self->regA = -2;
    return;
  }

  // carrega o programa .maq e obtém o ponto de entrada (PC inicial)
  int pc_inicio = so_carrega_programa(self, nome);
  if (pc_inicio < 0) {
    // erro na carga (nome inválido, arquivo inexistente, erro de montagem, ...)
    self->regA = -3;
    return;
  }

  // encontra um slot livre na tabela de processos
  int slot = -1;
  for (int i = 0; i < MAX_PROC; i++) {
    if (!self->proc[i].em_uso) { slot = i; break; }
  }
  if (slot < 0) {
    self->regA = -4; // sem espaço
    return;
  }

  // inicializa PCB do novo processo
  pcb_t *p = &self->proc[slot];
  p->em_uso   = true;
  p->pid      = pid_next++;
  p->estado   = P_PRONTO;
  p->A        = 0;
  p->X        = 0;
  p->ERRO     = 0;
  p->PC       = pc_inicio;
  p->term_base = escolhe_term_base_por_pid(p->pid);

  self->n_procs++;

  // retorna o pid do processo criado no registrador A da syscall
  self->regA = p->pid;
}

// implementação da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  if (self->idx_atual < 0) { self->regA = -1; return; }

  int alvo_pid = self->proc[self->idx_atual].X; // pid em X (0 = próprio)
  if (alvo_pid == 0) alvo_pid = self->proc[self->idx_atual].pid;

  int slot = -1;
  for (int i = 0; i < MAX_PROC; i++) {
    if (self->proc[i].em_uso && self->proc[i].pid == alvo_pid) { slot = i; break; }
  }
  if (slot < 0) { self->regA = -2; return; }

  self->proc[slot].estado = P_MORTO;
  self->proc[slot].em_uso = false;
  self->n_procs--;
  if (slot == self->idx_atual) self->idx_atual = -1; // será reescalonado
  self->regA = 0;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  // t2: deveria bloquear o processo se for o caso (e desbloquear na morte do esperado)
  // ainda sem suporte a processos, retorna erro -1
  console_printf("SO: SO_ESPERA_PROC não implementada");
  self->regA = -1;
}


// ---------------------------------------------------------------------
// CARGA DE PROGRAMA {{{1
// ---------------------------------------------------------------------

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}


// ---------------------------------------------------------------------
// ACESSO À MEMÓRIA DOS PROCESSOS {{{1
// ---------------------------------------------------------------------

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// t2: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

// vim: foldmethod=marker
