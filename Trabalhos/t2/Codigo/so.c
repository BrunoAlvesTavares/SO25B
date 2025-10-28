// so.c
// sistema operacional
// simulador de computador
// so25b


// INCLUDES 


#include "so.h"
#include "dispositivos.h"
#include "err.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"

#include <stdlib.h>
#include <stdbool.h>
#include "metricas.h"

// CONSTANTES E TIPOS 

// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50
#define QUANTUM_PADRAO 20
#define MAX_PROC 10


// 1 = Round-Robin simples
// 2 = Round-Robin com prioridade dinâmica
#define ESCALONADOR 1

typedef enum
{
  P_MORTO = 0,
  P_PRONTO,
  P_EXEC,
  P_BLOQ_LE,
  P_BLOQ_ES,
  P_BLOQ_WAIT
} p_estado_t;

typedef struct
{
  int pid;
  p_estado_t estado;
  int A, X, PC, ERRO;
  int term_base;
  bool em_uso;

  // NOVOS CAMPOS (Parte 2)
  int espera_dev;       // dispositivo que está aguardando (-1 se nenhum)
  int espera_pid;       // PID aguardado (ESPERA_PROC)
  int pend_ch;          // caractere pendente de escrita (SO_ESCR)
  int quantum;          // quantum máximo permitido ao processo
  int quantum_restante; // quantum restante (contador)
  float prioridade;     // 0.0..1.0 (quanto MENOR, maior a prioridade)
  int   t_exec_atual;   // tempo executado no ciclo atual (em "ticks" do timer)
  int num_preempcoes;  // número de preempções sofridas por este processo
} pcb_t;

static int pid_next = 1;

static int escolhe_term_base_por_pid(int pid)
{
  int s = (pid - 1) % 4;
  switch (s)
  {
  case 0:
    return D_TERM_A;
  case 1:
    return D_TERM_B;
  case 2:
    return D_TERM_C;
  default:
    return D_TERM_D;
  }
}

static int dev_teclado_ok(int base) { return base + TERM_TECLADO_OK; }
static int dev_teclado(int base) { return base + TERM_TECLADO; }
static int dev_tela_ok(int base) { return base + TERM_TELA_OK; }
static int dev_tela(int base) { return base + TERM_TELA; }

struct so_t
{
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  int regA, regX, regPC, regERRO; // cópia do estado da CPU

  pcb_t proc[MAX_PROC];
  int idx_atual;  // índice na tabela do processo em execução, -1 se nenhum
  int n_procs;    // quantidade de entradas em uso
  int ultimo_idx; // último índice escalonado (para round-robin)
  metricas_t *met;
  long tempo_ocioso;          // tempo total sem processo em execução
  int num_preempcoes_total;   // número total de preempções
};

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);
static int so_get_instr_count(so_t *self);



so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL)
    return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // tabela de processos
  self->idx_atual = -1;
  self->n_procs = 0;
for (int i = 0; i < MAX_PROC; i++)
{
  self->proc[i].em_uso = false;
  self->proc[i].estado = P_MORTO;
  self->proc[i].pid = 0;
  self->proc[i].A = 0;
  self->proc[i].X = 0;
  self->proc[i].PC = 0;
  self->proc[i].ERRO = 0;
  self->proc[i].term_base = D_TERM_A; // valor default; será definido ao criar
  self->proc[i].espera_dev = -1;
  self->proc[i].espera_pid = 0;
  self->proc[i].pend_ch = 0;
  self->proc[i].quantum = QUANTUM_PADRAO;
  self->proc[i].quantum_restante = QUANTUM_PADRAO;
  self->proc[i].prioridade   = 0.5f;
  self->proc[i].t_exec_atual = 0;
  self->proc[i].num_preempcoes = 0;
}

self->tempo_ocioso = 0;
self->num_preempcoes_total = 0;
self->ultimo_idx = -1;
self->met = metricas_cria();

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  console_printf("SO: salvando métricas em metricas.txt");
  metricas_salvar(self->met, "metricas.txt");
  metricas_destroi(self->met);
  free(self);
}


// TRATAMENTO DE INTERRUPÇÃO 


// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);
static bool pid_morto_ou_inexistente(so_t *self, int pid);
static void acorda_esperando_pid(so_t *self, int pid_morto);
// ADIÇÃO PARTE 3B helpers de prioridade
static void prio_inicializa_pcb(pcb_t *p);
static void prio_antes_de_bloquear(pcb_t *p);
static void prio_antes_de_preemptar(pcb_t *p);
static void prio_ao_despachar(pcb_t *p);

// ADIÇÃO PARTE 3: duas variantes do escalonador
static void so_escalona_rr(so_t *self);
static void so_escalona_prio(so_t *self);

// protótipos auxiliares
static int idx_do_pid(so_t *self, int pid);
static bool existem_processos_vivos_exceto(so_t *self, int pid_exceto);


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
  if (mem_le(self->mem, CPU_END_A, &A) != ERR_OK || mem_le(self->mem, CPU_END_PC, &PC) != ERR_OK || mem_le(self->mem, CPU_END_erro, &ERRO) != ERR_OK || mem_le(self->mem, 59, &X) != ERR_OK)
  {
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
  if (self->idx_atual >= 0)
  {
    pcb_t *p = &self->proc[self->idx_atual];
    p->A = A;
    p->X = X;
    p->PC = PC;
    p->ERRO = ERRO;
  }
}

static void so_trata_pendencias(so_t *self)
{
  // Trata desbloqueios de E/S e WAIT por PID
  for (int i = 0; i < MAX_PROC; i++)
  {
    pcb_t *p = &self->proc[i];
    if (!p->em_uso)
      continue;

    if (p->estado == P_BLOQ_LE && p->espera_dev >= 0)
    {
      int ok;
      if (es_le(self->es, dev_teclado_ok(p->espera_dev), &ok) == ERR_OK && ok)
      {
        int ch;
        if (es_le(self->es, dev_teclado(p->espera_dev), &ch) == ERR_OK)
        {
          p->A = ch;
          p->estado = P_PRONTO;
          p->espera_dev = -1;
        }
      }
    }

    if (p->estado == P_BLOQ_ES && p->espera_dev >= 0)
    {
      int ok;
      if (es_le(self->es, dev_tela_ok(p->espera_dev), &ok) == ERR_OK && ok)
      {
        if (es_escreve(self->es, dev_tela(p->espera_dev), p->pend_ch) == ERR_OK)
        {
          p->pend_ch = 0;
          p->espera_dev = -1;
          p->estado = P_PRONTO;
        }
      }
    }

    if (p->estado == P_BLOQ_WAIT)
    {
      // Caso 1: esperando um PID específico (>0)
      if (p->espera_pid > 0)
      {
        if (pid_morto_ou_inexistente(self, p->espera_pid))
        {
          p->espera_pid = 0;
          p->estado = P_PRONTO;
        }
      }
      // Caso 2: esperando "TODOS" (sinal especial -1)
      else if (p->espera_pid == -1)
      {
        // Desbloqueia quando não houver mais nenhum outro processo vivo
        if (!existem_processos_vivos_exceto(self, p->pid))
        {
          p->espera_pid = 0;
          p->estado = P_PRONTO;
        }
      }
    }
  }
}

static void so_escalona(so_t *self)
{
  // Se havia um processo executando, marca como PRONTO (a não ser que esteja bloqueado)
  if (self->idx_atual >= 0 && self->proc[self->idx_atual].estado == P_EXEC) {
    self->proc[self->idx_atual].estado = P_PRONTO;
  }

#if ESCALONADOR == 1
  so_escalona_rr(self);
#elif ESCALONADOR == 2
  so_escalona_prio(self);
#else
# error "ESCALONADOR deve ser 1 (RR) ou 2 (PRIO)"
#endif
}

__attribute__((unused))
static void so_escalona_rr(so_t *self)
{
  // Round-Robin simples: começa do próximo após o último
  int start = (self->ultimo_idx + 1 + MAX_PROC) % MAX_PROC;
  for (int k = 0; k < MAX_PROC; k++) {
    int i = (start + k) % MAX_PROC;
    if (self->proc[i].em_uso && self->proc[i].estado == P_PRONTO) {
      self->idx_atual = i;
      self->proc[i].estado = P_EXEC;
      self->ultimo_idx = i;
      return;
    }
  }
  self->idx_atual = -1;
}

__attribute__((unused))
static void so_escalona_prio(so_t *self)
{
  int melhor = -1;
  float menor_prio = 1e9f;
  int pid_atual = (self->idx_atual >= 0) ? self->proc[self->idx_atual].pid : -1;

  for (int i = 0; i < MAX_PROC; i++) {
    if (self->proc[i].em_uso && self->proc[i].estado == P_PRONTO) {
      // Evita reescolher o mesmo processo imediatamente
      if (self->proc[i].pid == pid_atual)
        continue;

      if (self->proc[i].prioridade < menor_prio) {
        menor_prio = self->proc[i].prioridade;
        melhor = i;
      }
    }
  }

  // se não achou outro, deixa o mesmo processo continuar
  if (melhor < 0 && pid_atual >= 0 && self->proc[self->idx_atual].estado == P_PRONTO)
    melhor = self->idx_atual;

  if (melhor >= 0) {
    self->idx_atual = melhor;
    self->proc[melhor].estado = P_EXEC;
    self->ultimo_idx = melhor;
  } else {
    self->idx_atual = -1;
  }
}

// PRIORIDADE DINÂMICA 
__attribute__((unused))
static void prio_inicializa_pcb(pcb_t *p)
{
  p->prioridade = 0.5f + (p->pid * 0.05f); // pequenas diferenças entre processos
  p->t_exec_atual = 0;
}

__attribute__((unused))
static void prio_recalcula(pcb_t *p)
{
  // t_exec = quanto do quantum ele realmente usou neste ciclo
  float t_exec = (float)p->t_exec_atual;
  float t_quantum = (float)p->quantum;

  if (t_quantum <= 0.f) t_quantum = 1.f; // evita div/0

  float frac = t_exec / t_quantum;      // 0.0 .. 1.0
  // prioridade = média móvel: (prio + frac) / 2
  p->prioridade = (p->prioridade + frac) * 0.5f;

  // prepara para o próximo ciclo
  p->t_exec_atual = 0;
}

static void prio_antes_de_bloquear(pcb_t *p)
{
#if ESCALONADOR == 2
  prio_recalcula(p);
#else
  (void)p;
#endif
}

__attribute__((unused))
static void prio_antes_de_preemptar(pcb_t *p)
{
#if ESCALONADOR == 2
  prio_recalcula(p);
#else
  (void)p;
#endif
}

__attribute__((unused))
static void prio_ao_despachar(pcb_t *p)
{
  // no PRIO, nada extra aqui além de zerar t_exec_atual (já fazemos no despacha)
  (void)p;
}

static int so_despacha(so_t *self)
{
  if (self->idx_atual < 0)
    return 1;

  pcb_t *p = &self->proc[self->idx_atual];

  // Só reseta o quantum se o processo anterior foi trocado
  if (p->quantum_restante <= 0 || p->estado != P_EXEC) {
    p->quantum_restante = p->quantum;
    p->t_exec_atual = 0;
  }

  if (mem_escreve(self->mem, CPU_END_A, p->A) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_PC, p->PC) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_erro, p->ERRO) != ERR_OK ||
      mem_escreve(self->mem, 59, p->X) != ERR_OK) {
    console_printf("SO: erro na escrita dos registradores");
    self->erro_interno = true;
    return 1;
  }

  return 0;
}

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  metricas_irq(self->met, irq);
  switch (irq)
  {
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
  if (ender != CPU_END_TRATADOR)
  {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK)
  {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  //  criar processo init (PID 1) e preparar a tabela de processos
  // limpa/normaliza a tabela
  for (int i = 0; i < MAX_PROC; i++)
  {
    self->proc[i].em_uso = false;
    self->proc[i].estado = P_MORTO;
    self->proc[i].pid = 0;
    self->proc[i].A = 0;
    self->proc[i].X = 0;
    self->proc[i].PC = 0;
    self->proc[i].ERRO = 0;
    self->proc[i].term_base = D_TERM_A;

    self->proc[i].espera_dev = -1;
    self->proc[i].espera_pid = 0;
    self->proc[i].pend_ch = 0;
    self->proc[i].quantum = QUANTUM_PADRAO;
    self->proc[i].quantum_restante = QUANTUM_PADRAO;
  }
  self->n_procs = 0;
  self->idx_atual = -1;
  self->ultimo_idx = -1;
  pid_next = 1;

  // carrega o init.maq
  ender = so_carrega_programa(self, "init.maq");
  if (ender != 100)
  {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  // cria PCB para o init
  int slot = 0;
  pcb_t *p = &self->proc[slot];
  p->em_uso = true;
  p->pid = pid_next++;
  p->estado = P_PRONTO;
  p->A = 0;
  p->X = 0;
  p->ERRO = 0;
  p->PC = ender; // ponto de entrada do init
  p->term_base = escolhe_term_base_por_pid(p->pid);
  self->n_procs = 1;
  p->espera_dev = -1;
  p->espera_pid = 0;
  p->pend_ch = 0;
  p->quantum = QUANTUM_PADRAO;
  p->quantum_restante = QUANTUM_PADRAO;
    // ADIÇÃO PARTE 3: prioridade do init
  p->prioridade   = 0.5f;
  p->t_exec_atual = 0;
  

  // Deixa o escalonador escolher (vai pegar o init agora).
  // Não escrevemos registradores da CPU diretamente; o despachante cuidará
  // de carregar o contexto do processo escolhido quando o SO retornar.
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  err_t err = self->regERRO;
  console_printf("SO: erro na CPU no processo atual: %s", err_nome(err));
if (self->idx_atual >= 0)
{
  int pid = self->proc[self->idx_atual].pid;
  self->proc[self->idx_atual].estado = P_MORTO;
  self->proc[self->idx_atual].em_uso = false;
  self->n_procs--;
  metricas_finalizou_proc(self->met, pid, so_get_instr_count(self));
  acorda_esperando_pid(self, pid);
  self->idx_atual = -1;

  // Salvar métricas quando o último processo morre
  if (self->n_procs <= 0) {
    console_printf("SO: todos os processos terminaram, salvando métricas...");
    metricas_salvar(self->met, "metricas.txt");
  }
}
}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // Rearma o timer
  es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0);
  es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);

  if (self->idx_atual < 0) {
    metricas_tick(self->met, -1, P_MORTO);
    self->tempo_ocioso += INTERVALO_INTERRUPCAO;
    return;
  }

  metricas_tick(self->met, self->idx_atual, self->proc[self->idx_atual].estado);

  pcb_t *p = &self->proc[self->idx_atual];
  p->quantum_restante -= INTERVALO_INTERRUPCAO;

  console_printf("SO: IRQ relogio, quantum restante = %d", p->quantum_restante);

if (p->quantum_restante <= 0) {
    // PREEMPÇÃO
    p->num_preempcoes++;
    self->num_preempcoes_total++;
    console_printf("SO: Quantum esgotado para o processo %d. Preempcao.", p->pid);

    // Atualiza métricas oficiais
    metricas_preempcao(self->met, p->pid);

    // marca o processo como PRONTO e devolve à fila
    p->estado = P_PRONTO;
    p->quantum_restante = p->quantum;

    // libera o slot atual para o escalonador escolher outro
    self->idx_atual = -1;
}

  // deixa o resto (pendências e escalonamento) seguir normalmente
  so_trata_pendencias(self);
  so_escalona(self);
  so_despacha(self);
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}


// CHAMADAS DE SISTEMA 


// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  // com processos, o reg A deve estar no descritor do processo corrente
   if (self->idx_atual < 0) return;              // não há processo corrente
  pcb_t *p = &self->proc[self->idx_atual];
  int id_chamada = p->A;                         // pega do PCB (como o colega)
  console_printf("SO: chamada de sistema %d", id_chamada);
  switch (id_chamada)
  {
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
    self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  if (self->idx_atual < 0)
  {
    self->regA = -1;
    return;
  }
  pcb_t *p = &self->proc[self->idx_atual];
  int base = p->term_base;

  int ok;
  if (es_le(self->es, dev_teclado_ok(base), &ok) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao estado do teclado");
    self->erro_interno = true;
    self->regA = -1;
    return;
  }

  if (!ok)
  {
      // vai bloquear a atualiza prioridade pelo tempo efetivo usado
  prio_antes_de_bloquear(p);

  // BLOQUEIA por leitura
  p->estado = P_BLOQ_LE;
  p->espera_dev = base;
  p->A = 0;                   // retorno no PCB
  self->idx_atual = -1;       // força o escalonador a trocar já
  return;
  }

  int ch;
  if (es_le(self->es, dev_teclado(base), &ch) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao teclado");
    self->erro_interno = true;
    self->regA = -1;
    return;
  }

  p->A = ch;
  self->regA = 0;
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
  if (self->idx_atual < 0)
  {
    self->regA = -1;
    return;
  }
  pcb_t *p = &self->proc[self->idx_atual];
  int base = p->term_base;
  int ch = p->X;

  int ok;
  if (es_le(self->es, dev_tela_ok(base), &ok) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao estado da tela");
    self->erro_interno = true;
    self->regA = -1;
    return;
  }

  if (!ok)
  {
     // vai bloquear a atualiza prioridade pelo tempo efetivo usado
  prio_antes_de_bloquear(p);

  // BLOQUEIA por escrita
p->estado = P_BLOQ_ES;
p->espera_dev = base;
p->pend_ch = ch;
p->A = 0;                   // retorno no PCB
self->idx_atual = -1;       // força troca
return;
  }

  if (es_escreve(self->es, dev_tela(base), ch) != ERR_OK)
  {
    console_printf("SO: problema na escrita na tela");
    self->erro_interno = true;
    self->regA = -1;
    return;
  }

  self->regA = 0;
}

// implementação da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  if (self->idx_atual < 0) {
    self->regA = -1;
    return;
  }

  pcb_t *pai = &self->proc[self->idx_atual];

  // lê o nome do executável do endereço em X do chamador
  char nome[128];
  if (!copia_str_da_mem(sizeof(nome), nome, self->mem, pai->X)) {
    pai->A = -2;
    return;
  }

  // carrega o programa
  int pc_inicio = so_carrega_programa(self, nome);
  if (pc_inicio < 0) {
    pai->A = -3;
    return;
  }

  // encontra um slot livre
  int slot = -1;
  for (int i = 0; i < MAX_PROC; i++) {
    if (!self->proc[i].em_uso) { slot = i; break; }
  }
  if (slot < 0) {
    pai->A = -4;
    return;
  }

  pcb_t *p = &self->proc[slot];
  p->em_uso = true;
  p->pid = pid_next++;
  p->estado = P_PRONTO;
  p->A = p->X = p->ERRO = 0;
  p->PC = pc_inicio;
  p->term_base = escolhe_term_base_por_pid(p->pid);
  p->espera_dev = -1;
  p->espera_pid = 0;
  p->pend_ch = 0;
  p->quantum = QUANTUM_PADRAO;
  p->quantum_restante = QUANTUM_PADRAO;
  p->prioridade   = 0.5f;
  p->t_exec_atual = 0;
  self->n_procs++;
  metricas_criou_proc(self->met, p->pid, so_get_instr_count(self));

  pai->A = p->pid;   // retorna PID do filho no registrador A do pai
}


// implementação da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  if (self->idx_atual < 0)
  {
    self->regA = -1;
    return;
  }

  int chamador_idx = self->idx_atual;
  int chamador_pid = self->proc[chamador_idx].pid;

  int alvo_pid = self->proc[chamador_idx].X; // pid em X (0 = próprio)
  if (alvo_pid == 0)
    alvo_pid = chamador_pid;

  // --- CASO ESPECIAL: init tentando se matar enquanto há filhos vivos
  if (chamador_pid == 1 && alvo_pid == 1) {
    if (existem_processos_vivos_exceto(self, 1)) {
      pcb_t *init = &self->proc[chamador_idx];
      prio_antes_de_bloquear(init);
      init->estado = P_BLOQ_WAIT;
      init->espera_pid = -1;     // aguardar "todos"
      init->A = 0;               // retorno OK no PCB
      self->idx_atual = -1;      // troca AGORA
      return;
    }
  }

  int slot = -1;
  for (int i = 0; i < MAX_PROC; i++)
  {
    if (self->proc[i].em_uso && self->proc[i].pid == alvo_pid)
    {
      slot = i;
      break;
    }
  }
  if (slot < 0)
  {
    self->regA = -2;
    return;
  }

  // Mata o alvo normalmente
  int pid_morto = self->proc[slot].pid;
  self->proc[slot].estado = P_MORTO;
  self->proc[slot].em_uso = false;
  self->n_procs--;

  // acorda quem esperava por esse PID
  acorda_esperando_pid(self, pid_morto);
  so_trata_pendencias(self);

  // Se quem morreu era o processo atual, força reescalonamento
  if (slot == self->idx_atual)
    self->idx_atual = -1;

  // Se nenhum processo restar, salvar métricas automaticamente
  if (self->n_procs <= 0) {
    console_printf("SO: todos os processos terminaram, salvando métricas...");
    metricas_salvar(self->met, "metricas.txt");
  }

  // retorno padrão OK
  self->proc[chamador_idx].A = 0;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X

static bool pid_morto_ou_inexistente(so_t *self, int pid)
{
  for (int i = 0; i < MAX_PROC; i++)
  {
    if (self->proc[i].em_uso && self->proc[i].pid == pid)
      return false;
  }
  return true;
}

__attribute__((unused))
static int idx_do_pid(so_t *self, int pid) {
  for (int i = 0; i < MAX_PROC; i++) {
    if (self->proc[i].em_uso && self->proc[i].pid == pid) return i;
  }
  return -1;
}

static bool existem_processos_vivos_exceto(so_t *self, int pid_exceto) {
  for (int i = 0; i < MAX_PROC; i++) {
    if (self->proc[i].em_uso && self->proc[i].estado != P_MORTO && self->proc[i].pid != pid_exceto) {
      return true;
    }
  }
  return false;
}

static void acorda_esperando_pid(so_t *self, int pid_morto)
{
  for (int i = 0; i < MAX_PROC; i++)
  {
    pcb_t *q = &self->proc[i];
    if (!q->em_uso)
      continue;
    if (q->estado == P_BLOQ_WAIT && q->espera_pid == pid_morto)
    {
      q->espera_pid = 0;
      q->estado = P_PRONTO;
    }
  }
}

static void so_chamada_espera_proc(so_t *self)
{
  if (self->idx_atual < 0)
  {
    self->regA = -1;
    return;
  }

  pcb_t *p = &self->proc[self->idx_atual];
  int alvo_pid = p->X;

  if (alvo_pid <= 0 || alvo_pid == p->pid)
  {
    self->regA = -2;
    return;
  }

if (pid_morto_ou_inexistente(self, alvo_pid)) {
  p->A = 0;
  return;
}

// BLOQUEIA aguardando alvo_pid
prio_antes_de_bloquear(p);
p->estado = P_BLOQ_WAIT;
p->espera_pid = alvo_pid;
p->A = 0;
self->idx_atual = -1;  // força reescalonamento
}


// CARGA DE PROGRAMA 


// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL)
  {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++)
  {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK)
    {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}

static int so_get_instr_count(so_t *self)
{
  int valor = 0;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &valor) != ERR_OK)
  {
    console_printf("SO: erro ao ler relógio (instruções)");
    return 0;
  }
  return valor;
}


// ACESSO À MEMÓRIA DOS PROCESSOS 


// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++)
  {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK)
    {
      return false;
    }
    if (caractere < 0 || caractere > 255)
    {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0)
    {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

// vim: foldmethod=marker