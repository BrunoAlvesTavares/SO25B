#include "metricas.h"
#include <stdlib.h>
#include <string.h>

metricas_t *metricas_cria(void) {
  metricas_t *m = calloc(1, sizeof(metricas_t));
  return m;
}

void metricas_destroi(metricas_t *m) {
  free(m);
}

static metrica_proc_t *getp(metricas_t *m, int pid) {
  for (int i=0; i<MAX_PROC_METRICAS; i++)
    if (m->proc[i].pid == pid) return &m->proc[i];
  for (int i=0; i<MAX_PROC_METRICAS; i++)
    if (m->proc[i].pid == 0) { m->proc[i].pid = pid; return &m->proc[i]; }
  return NULL;
}

void metricas_tick(metricas_t *m, int idx_proc, int estado_proc) {
  m->tempo_total++;
  if (idx_proc < 0) { m->tempo_ocioso++; return; }

  metrica_proc_t *p = getp(m, idx_proc);
  switch (estado_proc) {
    case 1: p->tempo_pronto++; break;
    case 2: p->tempo_exec++; break;
    default: p->tempo_bloq++; break;
  }
}

void metricas_irq(metricas_t *m, irq_t irq) {
  if (irq < N_IRQ) m->irq_count[irq]++;
}

void metricas_criou_proc(metricas_t *m, int pid, int tempo) {
  m->procs_criados++;
  metrica_proc_t *p = getp(m, pid);
  p->criado_em = tempo;
}

void metricas_finalizou_proc(metricas_t *m, int pid, int tempo) {
  metrica_proc_t *p = getp(m, pid);
  p->terminou_em = tempo;
}

void metricas_preempcao(metricas_t *m, int pid) {
  metrica_proc_t *p = getp(m, pid);
  p->preempcoes++;
  m->total_preempcoes++;
}

void metricas_transicao(metricas_t *m, int pid, int de, int para) {
  metrica_proc_t *p = getp(m, pid);
  if (de != para) {
    if (para == 1) p->trans_pronto++;
    if (para == 2) p->trans_exec++;
    if (para >= 3) p->trans_bloq++;
  }
}

void metricas_resposta(metricas_t *m, int pid, int delta) {
  metrica_proc_t *p = getp(m, pid);
  p->soma_resp += delta;
  p->n_resp++;
}

void metricas_salvar(metricas_t *m, const char *nome) {
  FILE *f = fopen(nome, "w");
  if (!f) return;

  fprintf(f, "=== Relatório de Métricas ===\n");
  fprintf(f, "Processos criados: %d\n", m->procs_criados);
  fprintf(f, "Tempo total: %d instruções\n", m->tempo_total);
  fprintf(f, "Tempo ocioso: %d\n", m->tempo_ocioso);
  fprintf(f, "Preempções totais: %d\n", m->total_preempcoes);

  fprintf(f, "\n--- Interrupções ---\n");
  for (int i=0; i<N_IRQ; i++)
    fprintf(f, "%s: %d\n", irq_nome(i), m->irq_count[i]);

  fprintf(f, "\n--- Por processo ---\n");
  for (int i=0; i<MAX_PROC_METRICAS; i++) {
    if (m->proc[i].pid == 0) continue;
    metrica_proc_t *p = &m->proc[i];
    int tempo_retorno = p->terminou_em - p->criado_em;
    float resp = (p->n_resp ? (float)p->soma_resp / p->n_resp : 0);
    fprintf(f,
      "PID %d: retorno=%d, pronto=%d, exec=%d, bloq=%d, preemp=%d, resp_med=%.2f\n",
      p->pid, tempo_retorno, p->tempo_pronto, p->tempo_exec,
      p->tempo_bloq, p->preempcoes, resp);
  }

  fclose(f);
}
