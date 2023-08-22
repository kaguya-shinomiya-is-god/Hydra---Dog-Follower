// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {

  // Inicializacao do controle
  private Joystick m_stick;
  private static Constants c;

  // Inicializacao da limelight
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  double limeX_Axis;
  double limeY_Axis;
  double limeArea;

  boolean precisionMode;
  boolean centralized;

  // Inicializacao dos Victors
  private VictorSPX mLeft1;
  private VictorSPX mLeft2;
  private VictorSPX mRight1;
  private VictorSPX mRight2;

  // Sensor Sharp
  AnalogInput sharp;
  double sharpReading;
  double sharpCalculated;

  @Override
  public void robotInit() {

    // Instanciando os elementos

    m_stick = new Joystick(0);

    mLeft1 = new VictorSPX(c.vicLeftFront);
    mLeft2 = new VictorSPX(c.vicLeftBack);
    mRight1 = new VictorSPX(c.vicRightFront);
    mRight2 = new VictorSPX(c.vicRightBack);

    // Configurando os motores

    mLeft2.follow(mLeft1);
    mRight2.follow(mRight1);

    mRight1.setInverted(true);
    mLeft1.setInverted(false);

    mLeft1.setNeutralMode(NeutralMode.Brake);
    mLeft2.setNeutralMode(NeutralMode.Brake);
    mRight1.setNeutralMode(NeutralMode.Brake);
    mRight2.setNeutralMode(NeutralMode.Brake);

    // Configurando as variaveis da limelight
    table = NetworkTableInstance.getDefault().getTable("limelght");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    // Configurando o Sharp
    sharp = new AnalogInput(0);
  }

  @Override
  public void autonomousPeriodic() {
    updateLimelight();
  }

  @Override
  public void teleopPeriodic() {
    updateLimelight();
    centralized = centralizedCheck();
    processSharp();

    // Verificando a centralizacao

    if (!centralized) {
      angularAdjustment();
    } else {
      dogFollowerLimeArea();
    }
  }

  private void updateLimelight() {
    // Atualizar os dados da Limelight

    limeX_Axis = tx.getDouble(0.0);
    limeY_Axis = ty.getDouble(0.0);
    limeArea = ta.getDouble(0.0);

    SmartDashboard.putNumber("Target X Axis", limeX_Axis);
    SmartDashboard.putNumber("Target Y Axis", limeY_Axis);
    SmartDashboard.putNumber("Target Area Percent", limeArea);
  }

  private boolean centralizedCheck() {
    // Checar se o elemento esta centralizado
    if (Math.abs(limeX_Axis) < c.kCT)
      return true;
    else
      return false;
  }

  private void angularAdjustment() {
    stopBabe();

    // Angular o robo de acordo com a diferenca do elemento em relacao ao
    // centro do eixo X (kCentrilized), estando sujeito a uma tolerancia
    // maxima (kCT) ou kCentralizedTolerance

    // CUIDADO POSSIVEL ERRO

    if ((limeX_Axis < -(c.kCentralized)) && (limeX_Axis > -(c.kCT)))
      goRight();
    else if ((limeX_Axis > (c.kCentralized)) && (limeX_Axis < (c.kCT)))
      goLeft();
    else
      stopBabe();
  }

  private void dogFollowerLimeArea() {
    stopBabe();

    // De acordo com valores de area de elemento da limelight,
    // iremos determinar se o robo ira pra frente ou para tras

    if ((limeArea < c.minArea) && (limeArea > 0.01))
      // Verifica se esta distante
      goFront();
    else if ((limeArea > c.minArea) && (limeArea < c.maxArea))
      // Verifica se esta na distancia segura
      stopBabe();
    else if (securityBack())
      // Verifica se esta seguro para ir para tras, caso nao
      // ira parar o movimento. (isso utilizando o sharp)
      goBack();
  }

  private void goFront() {
    mLeft1.set(ControlMode.PercentOutput, c.kSpd);
    mRight1.set(ControlMode.PercentOutput, c.kSpd);
  }

  private void goLeft() {
    mLeft1.set(ControlMode.PercentOutput, -c.kSpd);
    mRight1.set(ControlMode.PercentOutput, c.kSpd);
  }

  private void goRight() {
    mLeft1.set(ControlMode.PercentOutput, c.kSpd);
    mRight1.set(ControlMode.PercentOutput, -c.kSpd);
  }

  private void goBack() {
    mLeft1.set(ControlMode.PercentOutput, -c.kSpd);
    mRight1.set(ControlMode.PercentOutput, -c.kSpd);
  }

  private void stopBabe() {
    mLeft1.set(ControlMode.PercentOutput, 0);
    mRight1.set(ControlMode.PercentOutput, 0);
  }

  private boolean securityBack() {
    // Verificando se o sharp esta a uma distancia segura, o
    // c.kSecurityDistance
    if (sharpCalculated < c.kSecurityDistance)
      return false;
    else
      return true;
  }

  private void processSharp() {
    // Processando os valores do sharp, coletando e convertendo a
    // unidade de medida convencional
    sharpReading = sharp.getValue();
    sharpCalculated = (6762 / (sharpReading - 9)) - 4;

    SmartDashboard.putNumber("SharpRaw", sharpReading);
    SmartDashboard.putNumber("SharpCalculated", sharpCalculated);
  }
}
