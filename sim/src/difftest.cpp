#include "Emulator.h"
#include <assert.h>
#include <dlfcn.h>
#include <getopt.h>

static const char *img_file = "/home/lieat/Desktop/ysyxSoC/ysyx/sim/../prog/bin/mem/hello-mem.bin";
static const char *ref_so_file = "/home/lieat/Desktop/ysyxSoC/ysyx/sim/../prog/riscv32-nemu-interpreter-so";

static uint8_t* pmem;
uint8_t* guest_to_host(uint32_t paddr) { return pmem + (paddr - CONFIG_MBASE); }
uint32_t host_to_guest(uint8_t *haddr) { return haddr - pmem + CONFIG_MBASE; }

typedef struct 
{
  uint32_t gpr[32];
  uint32_t pc;
  uint32_t csr[4];
} CPU_state;

static CPU_state cpu;

void (*ref_difftest_memcpy)(uint32_t addr, void *buf, size_t n, bool direction) = NULL;
void (*ref_difftest_regcpy)(void *dut, bool direction) = NULL;
void (*ref_difftest_exec)(uint64_t n) = NULL;
void (*ref_difftest_skip)(uint64_t n) = NULL;
enum { DIFFTEST_TO_DUT, DIFFTEST_TO_REF };

void init_mem(){
  pmem = (uint8_t*)malloc(CONFIG_MSIZE);
  assert(pmem);
}

long load_img() {
  FILE *fp = fopen(img_file, "rb");
  assert(fp);

  fseek(fp, 0, SEEK_END);
  long size = ftell(fp);

  fseek(fp, 0, SEEK_SET);
  int ret = fread(guest_to_host(0x30000000), size, 1, fp);
  assert(ret == 1);

  fclose(fp);
  return size;
}

void init_difftest(){
  init_mem();
  long img_size = load_img();
  assert(ref_so_file != NULL);
  
  void *handle;
  handle = dlopen(ref_so_file, RTLD_LAZY);
  assert(handle);
  
  ref_difftest_memcpy = (void (*)(uint32_t addr, void *buf, size_t n, bool direction))(dlsym(handle, "difftest_memcpy"));
  assert(ref_difftest_memcpy);
  
  ref_difftest_regcpy = (void (*)(void *dut, bool direction))(dlsym(handle, "difftest_regcpy"));
  assert(ref_difftest_regcpy);

  ref_difftest_exec = (void (*)(uint64_t n))(dlsym(handle, "difftest_exec"));
  assert(ref_difftest_exec);

  ref_difftest_skip = (void (*)(uint64_t n))(dlsym(handle, "difftest_skip"));
  assert(ref_difftest_skip);

  void (*ref_difftest_init)() = (void (*)())(dlsym(handle, "difftest_init"));
  assert(ref_difftest_init);
  
  ref_difftest_init();
  ref_difftest_memcpy(CONFIG_MBASE, guest_to_host(CONFIG_MBASE), CONFIG_MSIZE, DIFFTEST_TO_REF);
  ref_difftest_regcpy(&cpu, DIFFTEST_TO_REF);
}

static int diff_valid 	 = 0;
static int diff_skip 	 = 0;
static int diff_sync     = 0;
static int diff_syncrd   = 0;
static int diff_syncdata = 0;
static bool diff_onemoreskip = false;

bool checkregs(CPU_state *ref_r, uint32_t pc) {
  bool check = true;
  for (int i = 0; i < 32; ++i){
    if (ref_r->gpr[i]!= cpu.gpr[i]){
      printf("for reg(%d), expected %x, but got %x.\n", i, ref_r->gpr[i], cpu.gpr[i]);
      check = false;
    }
  }
  if (ref_r->pc == pc){
    return check;
  }
  else {
    printf("PC expected %x but got %x\n", ref_r->pc, pc);
    return false;
  }
}

void ref_difftest_regset(CPU_state *ref){
  CPU_state ref_r;
  ref_difftest_regcpy(&ref_r,DIFFTEST_TO_DUT);
  for(int i = 0; i < 32; i++){
    ref->gpr[i] = ref_r.gpr[i];
  }
  ref->gpr[diff_syncrd] = diff_syncdata;
}

bool difftest_step() {
  if(diff_valid){
  CPU_state ref_r;
    if (diff_skip){
      diff_onemoreskip = false;
      ref_difftest_skip(1);
      if(diff_sync){
        ref_difftest_regset(&ref_r);
        ref_difftest_regcpy(&ref_r, DIFFTEST_TO_REF);
      }
      return true;
    }
    if(diff_onemoreskip){
      diff_onemoreskip = false;
      ref_difftest_skip(1);
      return true;
    }
    ref_difftest_skip(1);
    ref_difftest_regcpy(&ref_r, DIFFTEST_TO_DUT);
    if(!checkregs(&ref_r, cpu.pc)) return false;
    else return true;
  }
  else return true;
}
// ================================================================================================================================================
// DIFFTEST DPI-C
// ================================================================================================================================================

extern "C" void ebreak(){
}

extern "C" void difftest_dut_regs(uint32_t pc, uint32_t Z0,uint32_t ra, uint32_t sp, uint32_t gp, uint32_t tp, uint32_t t0, uint32_t t1, uint32_t t2,
uint32_t fp, uint32_t s1, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5,
uint32_t a6, uint32_t a7, uint32_t s2, uint32_t s3, uint32_t s4, uint32_t s5, uint32_t s6, uint32_t s7,
uint32_t s8, uint32_t s9, uint32_t s10, uint32_t a11, uint32_t t3, uint32_t t4, uint32_t t5, uint32_t t6){
  cpu.pc = pc;
  cpu.gpr[0] = Z0;  cpu.gpr[1] = ra;  cpu.gpr[2]  = sp; cpu.gpr[3]  = gp; cpu.gpr[4]  = tp; cpu.gpr[5]  = t0; cpu.gpr[6]  = t1; cpu.gpr[7]  = t2;
  cpu.gpr[8] = fp;  cpu.gpr[9] = s1;  cpu.gpr[10] = a0; cpu.gpr[11] = a1; cpu.gpr[12] = a2; cpu.gpr[13] = a3; cpu.gpr[14] = a4; cpu.gpr[15] = a5;
  cpu.gpr[16] = a6; cpu.gpr[17] = a7; cpu.gpr[18] = s2; cpu.gpr[19] = s3; cpu.gpr[20] = s4; cpu.gpr[21] = s5; cpu.gpr[22] = s6; cpu.gpr[23] = s7;
  cpu.gpr[24] = s8; cpu.gpr[25] = s9; cpu.gpr[26] = s10;cpu.gpr[27] = a11;cpu.gpr[28] = t3; cpu.gpr[29] = t4; cpu.gpr[30] = t5; cpu.gpr[31] = t6;
}

extern "C" void difftest_dut_sync(uint32_t diff_tag, uint32_t sync_data){
  diff_valid = (diff_tag % 256 - diff_tag % 128)/128;
  diff_skip          = (diff_tag % 128 - diff_tag % 64 )/64;
  diff_sync          = (diff_tag % 64  - diff_tag % 32 )/32;
  diff_syncrd        = diff_tag % 32;
  diff_syncdata      = sync_data;
}

