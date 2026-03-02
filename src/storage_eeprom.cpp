#include "storage_eeprom.h"

//--------------------------------- Lee EEPROM ---------------------------------------
float Load_EEPROM(int address)
{
  float value = 0.0f;
  EEPROM.get(address, value);
  if (isnan(value)) {  // Si el valor es NaN, significa que la EEPROM no estaba inicializada
    value = (address == ADD_SNS_VOLT_OFF_CAL || address == ADD_SNS_CURR_OFF_CAL ||
                address == ADD_OUT_CURR_OFF_CAL) ? 0.0 : 1.0;
    EEPROM.put(address, value);  // Guardar el valor por defecto en la EEPROM
    EEPROM.commit();
  }
  return value;
}

//--------------------------------- Graba EEPROM -------------------------------------
void Save_EEPROM(int address, float value) {
  EEPROM.put(address, value);                       // Guardar el valor en la EEPROM
  EEPROM.commit();                                 // Asegurar que los datos se escriban
}

//--------------------------------------- Carga de la EEPRON factores de Calibration ------------------------------------------
void Load_Calibration() {

  struct CalibrationData {
    int address;
    float &variable;
    const char *name;
    bool isOffset;  // Indica si es un OFF_CAL (true) o un FAC_CAL (false)
  };

  CalibrationData data[] = {
    {ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact, "Sns_Volt_Calib_Fact", false},
    {ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact, "Sns_Curr_Calib_Fact", false},
    {ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact, "Out_Curr_Calib_Fact", false},
    {ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs, "Sns_Volt_Calib_Offs", true},
    {ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs, "Sns_Curr_Calib_Offs", true},
    {ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs, "Out_Curr_Calib_Offs", true},
  };

  for (CalibrationData &d : data) {
    d.variable = Load_EEPROM(d.address);

    if (!d.isOffset) {  // Es un FAC_CAL (factor de calibración)
        if (d.variable < 0.9 || d.variable > 1.1) {
            d.variable = 1.0;  // Restaurar al valor por defecto
        }
    } else {  // Es un OFF_CAL (offset de calibración)
        if (d.variable < -0.1 || d.variable > 0.1) {
            d.variable = 0.0;  // Restaurar al valor por defecto
        }
    }
  }

}

//----------------------------------------- Validar Calibración y grabar en EEPROM --------------------------------------------
void Save_Calibration() {
  float eeprom_read_Cal;

  // Lista de direcciones y variables a verificar
  struct CalibrationData {
      int address;
      float &variable;
      const char *name;
  };

  CalibrationData data[] = {
      {ADD_SNS_VOLT_FAC_CAL, Sns_Volt_Calib_Fact, "Sns_Volt_Calib_Fact"},
      {ADD_SNS_CURR_FAC_CAL, Sns_Curr_Calib_Fact, "Sns_Curr_Calib_Fact"},
      {ADD_OUT_CURR_FAC_CAL, Out_Curr_Calib_Fact, "Out_Curr_Calib_Fact"},
      {ADD_SNS_VOLT_OFF_CAL, Sns_Volt_Calib_Offs, "Sns_Volt_Calib_Offs"},
      {ADD_SNS_CURR_OFF_CAL, Sns_Curr_Calib_Offs, "Sns_Curr_Calib_Offs"},
      {ADD_OUT_CURR_OFF_CAL, Out_Curr_Calib_Offs, "Out_Curr_Calib_Offs"},
  };

  // Verificar y guardar solo si el valor ha cambiado
  for (CalibrationData &d : data) {
    eeprom_read_Cal = Load_EEPROM(d.address);
      if (abs(d.variable - eeprom_read_Cal) > 0.0001) {
          Save_EEPROM(d.address, d.variable);
      }
  }
}
