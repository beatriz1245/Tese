#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================== OLED (SPI por software) ==================
#define OLED_LARGURA 128
#define OLED_ALTURA  64
#define OLED_MOSI    11  // DIN
#define OLED_CLK     13  // CLK
#define OLED_DC       8  // D/C
#define OLED_CS       9  // CS
#define OLED_RESET   10  // RES

Adafruit_SSD1306 ecra(OLED_LARGURA, OLED_ALTURA,
                      OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ================== Saídas de seleção de tensão ==================
const int pinos_saida[4] = {2, 3, 4, 5};

// ================== Ponte H (L298N) ==================
const int motor_pino1 = 6;
const int motor_pino2 = 7;
const int motor_ena   = -1; 

// ================== Encoder ==================
const int encoder_clk   = A0;
const int encoder_dt    = A1;
const int encoder_botao = 12;

// ================== Menu ==================
enum EstadoMenu { ECRA_PRINCIPAL, CONFIG_TENSAO, CONFIG_PULSO, CONFIG_GAP, CONFIG_ONDA };
const char  tipos_onda_iniciais[4] = {'M','B','A','P'};
const char* tipos_onda_nomes[4]    = {"Monophasic","Biphasic","Alternating","Polyphasic"};

EstadoMenu menu_actual = ECRA_PRINCIPAL;
int indice_menu = 0;
const int TOTAL_ITENS_MENU = 4;
const int ITENS_VISIVEIS   = 2;
int scroll_offset = 0;

// ================== Controlo & parâmetros ==================
int  nivel_tensao = 0;
int  bits_saida[4];
String estado_sistema = "ativo";
int  tipo_onda = 0;   

// Combinações dos 4 bits para os 6 níveis de tensão
const uint8_t configuracao_tensoes[6][4] = {
  {1,1,0,1}, // 5V
  {0,0,0,1}, // 7V
  {1,0,1,0}, // 9V
  {1,1,0,0}, // 11V
  {0,0,1,0}, // 13V
  {0,1,0,0}  // 15V
};
const int QTD_NIVEIS_TENSAO = sizeof(configuracao_tensoes)/sizeof(configuracao_tensoes[0]);

const float TENSAO_BASE = 5.0f;
const float INCREMENTO_TENSAO = 2.0f;

unsigned long duracao_pulso = 10;   // ms
unsigned long gap           = 300;  // ms

const unsigned long PULSO_MIN = 5,   PULSO_MAX = 100;  // ms
const unsigned long GAP_MIN   = 50,  GAP_MAX   = 2000;  // ms
const int           TIPO_ONDA_MIN = 0, TIPO_ONDA_MAX = 3;

const float TENSAO_MINIMA = 5.0f;
const float TENSAO_MAXIMA = 15.0f;

// Ponte H 
unsigned long tempo_anterior_ponte = 0;
int estado_ponte = 0;


void actualizar_ecra();
void mostrar_menu_principal();
void mostrar_config_tensao();
void mostrar_config_pulso();
void mostrar_config_gap();
void mostrar_config_wave();

void executar_rotacao_esquerda();
void executar_rotacao_direita();
void processar_encoder();
void resetar_estado_ponte();
void controlar_ponte_h();
void controlar_monofasico(unsigned long t);
void controlar_bifasico(unsigned long t);
void controlar_alternante(unsigned long t);
void controlar_polifasico(unsigned long t);

float obter_tensao(int indice) {
  if (indice < 0) indice = 0;
  if (indice >= QTD_NIVEIS_TENSAO) indice = QTD_NIVEIS_TENSAO - 1;
  return TENSAO_BASE + (indice * INCREMENTO_TENSAO);
}
String formatar_voltagem(float tensao) { return String(tensao, 1) + "V"; }
void calcular_scroll() {
  if (indice_menu < scroll_offset) scroll_offset = indice_menu;
  else if (indice_menu >= scroll_offset + ITENS_VISIVEIS) scroll_offset = indice_menu - ITENS_VISIVEIS + 1;
  if (scroll_offset < 0) scroll_offset = 0;
  if (scroll_offset > TOTAL_ITENS_MENU - ITENS_VISIVEIS) scroll_offset = TOTAL_ITENS_MENU - ITENS_VISIVEIS;
}

// ------------------ Encoder ------------------
void executar_rotacao_esquerda() {
  switch (menu_actual) {
    case ECRA_PRINCIPAL:
      indice_menu = (indice_menu - 1 + TOTAL_ITENS_MENU) % TOTAL_ITENS_MENU;
      calcular_scroll(); actualizar_ecra(); break;
    case CONFIG_TENSAO:
      if (nivel_tensao > 0) { nivel_tensao--; actualizar_ecra(); }
      break;
    case CONFIG_PULSO:
      if (duracao_pulso > PULSO_MIN) { duracao_pulso = duracao_pulso >= 5 ? duracao_pulso - 5 : PULSO_MIN; actualizar_ecra(); }
      break;
    case CONFIG_GAP:
      if (gap > GAP_MIN) { gap = gap >= 50 ? gap - 50 : GAP_MIN; actualizar_ecra(); }
      break;
    case CONFIG_ONDA:
      if (tipo_onda > TIPO_ONDA_MIN) { tipo_onda--; actualizar_ecra(); resetar_estado_ponte(); }
      break;
  }
}
void executar_rotacao_direita() {
  switch (menu_actual) {
    case ECRA_PRINCIPAL:
      indice_menu = (indice_menu + 1) % TOTAL_ITENS_MENU;
      calcular_scroll(); actualizar_ecra(); break;
    case CONFIG_TENSAO:
      if (nivel_tensao < QTD_NIVEIS_TENSAO - 1) { nivel_tensao++; actualizar_ecra(); }
      break;
    case CONFIG_PULSO:
      if (duracao_pulso < PULSO_MAX) { duracao_pulso = duracao_pulso + 5 <= PULSO_MAX ? duracao_pulso + 5 : PULSO_MAX; actualizar_ecra(); }
      break;
    case CONFIG_GAP:
      if (gap < GAP_MAX) { gap = gap + 50 <= GAP_MAX ? gap + 50 : GAP_MAX; actualizar_ecra(); }
      break;
    case CONFIG_ONDA:
      if (tipo_onda < TIPO_ONDA_MAX) { tipo_onda++; actualizar_ecra(); resetar_estado_ponte(); }
      break;
  }
}
void processar_encoder() {
  static int estado_clk_anterior = HIGH;
  static unsigned long t_last = 0;
  const unsigned long debounce_ms = 3;

  unsigned long agora = millis();
  if (agora - t_last < debounce_ms) return;

  int clk = digitalRead(encoder_clk);
  int dt  = digitalRead(encoder_dt);

  if (clk != estado_clk_anterior && clk == LOW) {
    if (dt != clk) executar_rotacao_direita();
    else           executar_rotacao_esquerda();
    t_last = agora;
  }
  estado_clk_anterior = clk;

  // Botão do encoder
  static unsigned long t_btn = 0;
  static bool btn_prev = HIGH;
  bool btn = digitalRead(encoder_botao);
  if (agora - t_btn > 250) {
    if (btn == LOW && btn_prev == HIGH) {
      switch (menu_actual) {
        case ECRA_PRINCIPAL:
          if      (indice_menu == 0) menu_actual = CONFIG_TENSAO;
          else if (indice_menu == 1) menu_actual = CONFIG_PULSO;
          else if (indice_menu == 2) menu_actual = CONFIG_GAP;
          else                       menu_actual = CONFIG_ONDA;
          actualizar_ecra(); break;
        default:
          menu_actual = ECRA_PRINCIPAL; actualizar_ecra(); break;
      }
      t_btn = agora;
    }
  }
  btn_prev = btn;
}

// ------------------ Ponte H – formas de onda ------------------
void resetar_estado_ponte() {
  digitalWrite(motor_pino1, LOW);
  digitalWrite(motor_pino2, LOW);
  estado_ponte = 0;                
  tempo_anterior_ponte = millis();
}
void controlar_monofasico(unsigned long t) {
  switch (estado_ponte) {
    case 0: // pulso +
      digitalWrite(motor_pino1, HIGH);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 1; tempo_anterior_ponte = t; }
      break;
    case 1: // gap
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= gap) { estado_ponte = 0; tempo_anterior_ponte = t; }
      break;
  }
}
void controlar_bifasico(unsigned long t) {
  switch (estado_ponte) {
    case 0: // +
      digitalWrite(motor_pino1, HIGH);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 1; tempo_anterior_ponte = t; }
      break;
    case 1: // -
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, HIGH);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 2; tempo_anterior_ponte = t; }
      break;
    case 2: // gap
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= gap) { estado_ponte = 0; tempo_anterior_ponte = t; }
      break;
  }
}
void controlar_alternante(unsigned long t) {
  switch (estado_ponte) {
    case 0: // +
      digitalWrite(motor_pino1, HIGH);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 1; tempo_anterior_ponte = t; }
      break;
    case 1: // gap
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= gap) { estado_ponte = 2; tempo_anterior_ponte = t; }
      break;
    case 2: // -
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, HIGH);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 3; tempo_anterior_ponte = t; }
      break;
    case 3: // gap
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= gap) { estado_ponte = 0; tempo_anterior_ponte = t; }
      break;
  }
}
void controlar_polifasico(unsigned long t) {
  switch (estado_ponte) {
    case 0: // + 
      digitalWrite(motor_pino1, HIGH);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 1; tempo_anterior_ponte = t; }
      break;
    case 1: // -
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, HIGH);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 2; tempo_anterior_ponte = t; }
      break;
    case 2: // +
      digitalWrite(motor_pino1, HIGH);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= duracao_pulso) { estado_ponte = 3; tempo_anterior_ponte = t; }
      break;
    case 3: // gap
      digitalWrite(motor_pino1, LOW);
      digitalWrite(motor_pino2, LOW);
      if (t - tempo_anterior_ponte >= gap) { estado_ponte = 0; tempo_anterior_ponte = t; }
      break;
  }
}
void controlar_ponte_h() {
  unsigned long t = millis();
  switch (tipo_onda) {
    case 0: controlar_monofasico(t);   break;
    case 1: controlar_bifasico(t);     break;
    case 2: controlar_alternante(t);   break;
    case 3: controlar_polifasico(t);   break;
  }
}

// ------------------ UI ------------------
void mostrar_menu_principal() {
  ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0); ecra.println("MAIN MENU");
  ecra.drawLine(0,10,127,10,SSD1306_WHITE);

  const char* itens_menu[TOTAL_ITENS_MENU] = {"Voltage","Pulse width","Gap","Pulse type"};
  if (indice_menu < 0 || indice_menu >= TOTAL_ITENS_MENU) indice_menu = 0;

  for (int i = 0; i < ITENS_VISIVEIS; i++) {
    int item_index = scroll_offset + i; if (item_index >= TOTAL_ITENS_MENU) break;
    int y = 18 + i*16;
    ecra.setCursor(10,y); ecra.setTextSize(1);
    if (item_index == indice_menu) {
      ecra.fillRect(8,y-2,112,14,SSD1306_WHITE);
      ecra.setTextColor(SSD1306_BLACK); ecra.print(">"); ecra.print(itens_menu[item_index]);
      ecra.setTextColor(SSD1306_WHITE);
    } else {
      ecra.print(" "); ecra.print(itens_menu[item_index]);
    }
  }
  if (scroll_offset > 0) ecra.drawTriangle(120,15,124,15,122,12,SSD1306_WHITE);
  if (scroll_offset + ITENS_VISIVEIS < TOTAL_ITENS_MENU) ecra.drawTriangle(120,45,124,45,122,48,SSD1306_WHITE);

  ecra.setCursor(0,56); ecra.setTextSize(1);
  ecra.print("V:"); ecra.print(obter_tensao(nivel_tensao),0);
  ecra.print("V W:"); ecra.print(duracao_pulso);
  ecra.print("ms P:"); ecra.print(tipos_onda_iniciais[tipo_onda]);
}
void mostrar_config_tensao() {
  ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0); ecra.println("VOLTAGE");
  ecra.drawLine(0,10,127,10,SSD1306_WHITE);

  float V = obter_tensao(nivel_tensao);
  ecra.setTextSize(3); ecra.setCursor(0,16); ecra.print(V,0);
  ecra.setTextSize(2); ecra.setCursor(48,22); ecra.print("V");

  ecra.setTextSize(1); ecra.setCursor(0,42);
  ecra.print("Range: "); ecra.print(TENSAO_MINIMA,0); ecra.print("-"); ecra.print(TENSAO_MAXIMA,0); ecra.print("V");

  if (nivel_tensao == 0) { ecra.setCursor(90,16); ecra.print("MIN"); }
  else if (nivel_tensao == QTD_NIVEIS_TENSAO-1) { ecra.setCursor(90,16); ecra.print("MAX"); }

  // barra de progresso sem map() float
  float frac = (V - TENSAO_MINIMA) / (TENSAO_MAXIMA - TENSAO_MINIMA);
  if (frac < 0) frac = 0; if (frac > 1) frac = 1;
  int largura = (int)(frac * 126.0f + 0.5f);
  ecra.drawRect(0,52,128,10,SSD1306_WHITE);
  ecra.fillRect(1,53,largura,8,SSD1306_WHITE);
}
void mostrar_config_pulso() {
  ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0); ecra.println("PULSE WIDTH");
  ecra.drawLine(0,10,127,10,SSD1306_WHITE);

  ecra.setTextSize(3); ecra.setCursor(0,16); ecra.print(duracao_pulso);
  ecra.setTextSize(1); ecra.setCursor(0,32); ecra.print("ms");

  ecra.setCursor(0,42); ecra.print("Range: "); ecra.print(PULSO_MIN); ecra.print("-"); ecra.print(PULSO_MAX); ecra.print("ms");

  if (duracao_pulso == PULSO_MIN) { ecra.setCursor(90,16); ecra.print("MIN"); }
  else if (duracao_pulso == PULSO_MAX) { ecra.setCursor(90,16); ecra.print("MAX"); }

  float frac = (float)(duracao_pulso - PULSO_MIN) / (float)(PULSO_MAX - PULSO_MIN);
  if (frac < 0) frac = 0; if (frac > 1) frac = 1;
  int largura = (int)(frac * 126.0f + 0.5f);
  ecra.drawRect(0,52,128,10,SSD1306_WHITE);
  ecra.fillRect(1,53,largura,8,SSD1306_WHITE);
}
void mostrar_config_gap() {
  ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0); ecra.println("GAP");
  ecra.drawLine(0,10,127,10,SSD1306_WHITE);

  ecra.setTextSize(3); ecra.setCursor(0,16); ecra.print(gap);
  ecra.setTextSize(1); ecra.setCursor(0,32); ecra.print("ms");

  ecra.setCursor(0,42); ecra.print("Range: "); ecra.print(GAP_MIN); ecra.print("-"); ecra.print(GAP_MAX); ecra.print("ms");

  if (gap == GAP_MIN) { ecra.setCursor(90,16); ecra.print("MIN"); }
  else if (gap == GAP_MAX) { ecra.setCursor(90,16); ecra.print("MAX"); }

  float frac = (float)(gap - GAP_MIN) / (float)(GAP_MAX - GAP_MIN);
  if (frac < 0) frac = 0; if (frac > 1) frac = 1;
  int largura = (int)(frac * 126.0f + 0.5f);
  ecra.drawRect(0,52,128,10,SSD1306_WHITE);
  ecra.fillRect(1,53,largura,8,SSD1306_WHITE);
}
void mostrar_config_wave() {
  ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0); ecra.println("PULSE TYPE");
  ecra.drawLine(0,10,127,10,SSD1306_WHITE);

  ecra.setCursor(0,20); ecra.print("TYPE: ");
  ecra.setCursor(0,37); ecra.println(tipos_onda_nomes[tipo_onda]);

  ecra.setCursor(0,56); ecra.print("Options: ");
  int x = 48;
  for (int i = 0; i < 4; i++) {
    ecra.setCursor(x,56);
    if (i == tipo_onda) { ecra.setTextColor(SSD1306_BLACK); ecra.fillRect(x-1,55,7,9,SSD1306_WHITE); }
    else                { ecra.setTextColor(SSD1306_WHITE); }
    ecra.print(tipos_onda_iniciais[i]); x += 10;
  }
  ecra.setTextColor(SSD1306_WHITE);
}
void actualizar_ecra() {
  ecra.clearDisplay();
  switch (menu_actual) {
    case ECRA_PRINCIPAL:  mostrar_menu_principal(); break;
    case CONFIG_TENSAO:   mostrar_config_tensao();  break;
    case CONFIG_PULSO:    mostrar_config_pulso();   break;
    case CONFIG_GAP:      mostrar_config_gap();     break;
    case CONFIG_ONDA:     mostrar_config_wave();    break;
  }
  ecra.display();
}

// ------------------ Setup ------------------
void setup() {
  for (int i = 0; i < 4; i++) { pinMode(pinos_saida[i], OUTPUT); digitalWrite(pinos_saida[i], LOW); }

  pinMode(motor_pino1, OUTPUT);
  pinMode(motor_pino2, OUTPUT);
  if (motor_ena >= 0) { pinMode(motor_ena, OUTPUT); digitalWrite(motor_ena, HIGH); }

  pinMode(encoder_clk, INPUT_PULLUP);
  pinMode(encoder_dt,  INPUT_PULLUP);
  pinMode(encoder_botao, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);

  if (!ecra.begin(SSD1306_SWITCHCAPVCC)) { delay(1000); ecra.begin(SSD1306_SWITCHCAPVCC); }

  ecra.clearDisplay(); ecra.setTextSize(1); ecra.setTextColor(SSD1306_WHITE);
  ecra.setCursor(0,0);
  ecra.println("System started");
  ecra.println("Menu: 4 options");
  ecra.println("Encoder: browse");
  ecra.println("Arrows: more options");
  ecra.display();
  delay(1200);

  actualizar_ecra();

  digitalWrite(motor_pino1, LOW);
  digitalWrite(motor_pino2, LOW);
  tempo_anterior_ponte = millis();
  estado_ponte = 0; 
}
void loop() {
  processar_encoder();

  // Limites defensivos
  if (nivel_tensao < 0) nivel_tensao = 0;
  if (nivel_tensao >= QTD_NIVEIS_TENSAO) nivel_tensao = QTD_NIVEIS_TENSAO - 1;
  if (tipo_onda < TIPO_ONDA_MIN) tipo_onda = TIPO_ONDA_MIN;
  if (tipo_onda > TIPO_ONDA_MAX) tipo_onda = TIPO_ONDA_MAX;

  // Atualiza os 4 bits de seleção de tensão
  for (int i = 0; i < 4; i++) {
    bits_saida[i] = configuracao_tensoes[nivel_tensao][i];
    digitalWrite(pinos_saida[i], bits_saida[i] ? HIGH : LOW);
  }

  
  controlar_ponte_h();
}
