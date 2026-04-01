#include "app/app_calibration_context.h"
#include "config/system_constants.h"
#define Sns_Volt_Calib_Fact (app_calibration_sns_volt_factor_ref())
#define Sns_Volt_Calib_Offs (app_calibration_sns_volt_offset_ref())
#define Sns_Curr_Calib_Fact (app_calibration_sns_curr_factor_ref())
#define Sns_Curr_Calib_Offs (app_calibration_sns_curr_offset_ref())
#define Out_Curr_Calib_Fact (app_calibration_out_curr_factor_ref())
#define Out_Curr_Calib_Offs (app_calibration_out_curr_offset_ref())
#define Temp_Calib_Fact (app_calibration_temp_factor_ref())


#include "storage_eeprom.h"

//--------------------------------- Lee EEPROM ---------------------------------------
float Load_EEPROM(int address)
{
  float value = 0.0f;
  EEPROM.get(address, value);
  if (isnan(value)) {  // Si el valor es NaN, significa que la EEPROM no estaba inicializada
    value = (address == ADD_SNS_VOLT_OFF_CAL || address == ADD_SNS_CURR_OFF_CAL ||
                address == ADD_OUT_CURR_OFF_CAL) ? 0.0f : 1.0f;
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
    {ADD_TEMP_FAC_CAL, Temp_Calib_Fact, "Temp_Calib_Fact", false},
  };

  for (CalibrationData &d : data) {
    d.variable = Load_EEPROM(d.address);

    if (!d.isOffset) {  // Es un FAC_CAL (factor de calibracion)
      float minFactor = CAL_FACTOR_MIN;
      float maxFactor = CAL_FACTOR_MAX;
      if (d.address == ADD_TEMP_FAC_CAL) {
        minFactor = TEMP_CAL_FACTOR_MIN;
        maxFactor = TEMP_CAL_FACTOR_MAX;
      }

      if (d.variable < minFactor || d.variable > maxFactor) {
        d.variable = 1.0;  // Restaurar al valor por defecto
      }
    } else {  // Es un OFF_CAL
      float minOffset = -0.1f;
      float maxOffset = 0.1f;

      // Out_Curr_Calib_Offs se almacena en mA; los demas offsets en unidades base (V o A)
      if (d.address == ADD_OUT_CURR_OFF_CAL) {
        minOffset = -100.0f;
        maxOffset = 100.0f;
      }

      if (d.variable < minOffset || d.variable > maxOffset) {
        d.variable = 0.0f;  // Restaurar al valor por defecto
      }
    }
  }

}

//----------------------------------------- Validar Calibracion y grabar en EEPROM --------------------------------------------
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
      {ADD_TEMP_FAC_CAL, Temp_Calib_Fact, "Temp_Calib_Fact"},
  };

  // Verificar y guardar solo si el valor ha cambiado
  for (CalibrationData &d : data) {
    eeprom_read_Cal = Load_EEPROM(d.address);
      if (abs(d.variable - eeprom_read_Cal) > 0.0001) {
          Save_EEPROM(d.address, d.variable);
      }
  }
}

