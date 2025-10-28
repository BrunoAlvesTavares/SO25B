#ifndef METRICAS_H
#define METRICAS_H

#include "irq.h"
#include "err.h"
#include <stdio.h>

#define MAX_PROC_METRICAS 16

typedef struct {
  int pid;
  int criado_em;
  int terminou_em;
  int tempo_pronto;
  int tempo_exec;
  int tempo_bloq;
  int trans_pronto;
  int trans_exec;
  int trans_bloq;
  int preempcoes;
  int soma_resp;
  int n_resp;
} metrica_proc_t;

typedef struct {
  int procs_criados;
  int tempo_total;
  int tempo_ocioso;
  int irq_count[N_IRQ];
  int total_preempcoes;
  metrica_proc_t proc[MAX_PROC_METRICAS];
} metricas_t;

// criação e destruição
metricas_t *metricas_cria(void);
void metricas_destroi(metricas_t *m);

// registro de eventos
void metricas_tick(metricas_t *m, int idx_proc, int estado_proc);
void metricas_irq(metricas_t *m, irq_t irq);
void metricas_criou_proc(metricas_t *m, int pid, int tempo);
void metricas_finalizou_proc(metricas_t *m, int pid, int tempo);
void metricas_preempcao(metricas_t *m, int pid);
void metricas_transicao(metricas_t *m, int pid, int de, int para);
void metricas_resposta(metricas_t *m, int pid, int delta);

// relatório
void metricas_salvar(metricas_t *m, const char *nome_arquivo);

#endif
