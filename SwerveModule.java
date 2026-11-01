package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

/**
 * Swerve'in tek bir köşesi, yani tek bir modül.
 *
 * Üç parçadan oluşuyor:
 *   1. Drive motoru : tekerleği döndürüp robotu iter
 *   2. Rotation motoru: tekerleğin baktığı yönü belirler
 *   3. CANcoder : tekerleğin gerçek açısını her zaman bilir

 */
public class SwerveModule extends SubsystemBase {
    /** İleri/geri süren drive motorunun SparkMax'ı. */
    private final SparkMax driveMotor;

    /** Tekerleği çeviren rotation motorunun SparkMax'ı. */
    private final SparkMax rotationMotor;

    /** Drive motorunun kendi encoder'ı. Gidilen mesafeyi sayar. */
    private final RelativeEncoder driveEncoder;

    /** Rotation motorunun kendi encoder'ı. Direksiyon açısındaki değişimi sayar. */
    private final RelativeEncoder rotationEncoder;

    /** Tekerleğin gerçek açısını veren mutlak encoder. */
    private final CANcoder canCoder;

    /**  Tekerlek düz ileri bakarken açı 0 çıksın diye. */
    private final double canCoderOffsetRadians;

    /** Tekerleği hedef açıya götüren */
    private final PIDController rotationPIDController;

    /** Tekerleği hedef hıza getiren  */
    private final PIDController drivePIDController;

    /**
     * Modülü verilen donanım ID'leri ve ayarlarla kurar.
     *
     * @param driveMotorID       drive SparkMax'ının CAN ID'si
     * @param rotationMotorID    rotation SparkMax'ının CAN ID'si
     * @param canCoderID         CANcoder'ın CAN ID'si
     * @param canCoderOffsetRadians  CANcoder kalibrasyon offset'i (radyan)
     * @param isDriveInverted    drive motorunun yönü ters çevrilsin mi
     * @param x                  rotation PID'nin P katsayısı
     */
    public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID,
      double canCoderOffsetRadians,
      boolean isDriveInverted,
      double x
    ) {
      // NEO kullandığımız için kBrushless
      driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

      // İki motora da şunlar uygulanıyor:
      // - Brake modu: güç yokken motor direniyor, robot yamulup kaymıyor
      // - Conversion factor: motor dönüşünü metre ya da radyana çeviriyor
      configureMotor(driveMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.drivePositionConversionFactor,
                      DrivetrainConstants.driveVelocityConversionFactor
      );
      configureMotor(rotationMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.rotationPositionConversionFactor,
                      DrivetrainConstants.rotationVelocityConversionFactor
      );

      // Önceki çalıştırmadan kalan hata bayraklarını sil
      driveMotor.clearFaults();
      rotationMotor.clearFaults();

      // --- PID'ler ---
      // Drive PID tekerlek hızını hedef hıza yaklaştırır
      drivePIDController = new PIDController(DrivetrainConstants.kPDrive, DrivetrainConstants.kIDrive, DrivetrainConstants.kDDrive);

      // Rotation PID tekerlek açısını hedef açıya yaklaştırır.
      // P değeri parametre olarak geliyor, böylece her modüle ayrı değer verilebilir (SwerveSubsystem'dan geliyor).
      rotationPIDController = new PIDController(x, DrivetrainConstants.kIRotation, DrivetrainConstants.kDRotation);
      rotationPIDController.setTolerance(DrivetrainConstants.kToleranceRotation);

      // -180 ile +180 aynı noktadır. Bunu PID'ye söylemezsek tekerlek bazen
      // kestirmeyi bırakıp uzun yoldan döner.
      rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // --- Encoder'lar ---
      // CANcoder mutlak konumu verir, güç gitse de kaybolmaz
      canCoder = new CANcoder(canCoderID);
      this.canCoderOffsetRadians = canCoderOffsetRadians;
      configureCanCoder();

      // SparkMax'ın bağıl encoder'ları. Açılışta CANcoder'a göre ayarlanıyor.
      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    /**
     * SparkMax'a yön, idle modu ve encoder dönüşüm çarpanlarını yazar.
     *
     * @param motorController        ayarlanacak SparkMax
     * @param isInverted             motor yönü ters mi
     * @param idleMode               kBrake (direnir) ya da kCoast (serbest döner)
     * @param positionConversionFactor  motor dönüşünü gerçek konum birimine çeviren çarpan
     * @param velocityConversionFactor  motor RPM'ini gerçek hız birimine çeviren çarpan
     */
    public void configureMotor(
      SparkMax motorController,
      boolean isInverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .inverted(isInverted)
          .idleMode(idleMode);
        config.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(velocityConversionFactor);

          motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      }

    /**
     * CANcoder ayarları. Değerler 0 ile 1 arasında (bir tam tur) gelsin, saat
     * yönünün tersi de pozitif olsun diye ayarlanyor
     */
    public void configureCanCoder(){
      CANcoderConfiguration config = new CANcoderConfiguration();

      // Değer 0-1 aralığında sarsın. Radyana çevirmek böyle kolay oluyor.
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      // Saat yönünün tersi pozitif (WPILib ile aynı)
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      // Ayarı CANcoder'a gönder
      canCoder.getConfigurator().apply(config);
    }

    /**
     * Modülün hedef durumunu (hız ve açı) uygular.
     *
     * İstenen hız sıfıra çok yakınsa modül tamamen durur.
     * Sürücü joystick'e dokunmuyorken tekerlekler rastgele açılara dönüp titremesin,
     * olduğu yerde kalsın diye.
     *
     * @param state hedef SwerveModuleState (hız m/s, açı radyan)
     */
    public void setState(SwerveModuleState state){
      // Hız neredeyse sıfırsa dur ve çık. Robot dururken tekerlekler oynamasın.
      if (Math.abs(state.speedMetersPerSecond) < 0.0001){
        stop();
        return;
      }

      // Tekerleği PID ile hedef açıya çevir
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), state.angle.getRadians()));

      // Tekerleği PID ile hedef hızda döndür
      driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
    }

    /**
     * Tekerleğin 90 dereceden fazla dönmesini engelleyen optimizasyon.
     *
     
     * @param desiredState  hedef durum
     * @param currentAngle  tekerleğin şu anki açısı
     * @return optimize edilmiş durum 
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90){
        return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
      }
      else{
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    /**
     * Modülün şu anki durumunu verir.
     *
     * @return şu anki hız (m/s) ve açıyla (radyan) bir SwerveModuleState
     */
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
    }

    /**
     * Rotation motorunun bağıl encoder'ını CANcoder'ın gösterdiği açıya eşitler.
     * Başlangıçta bir kez çağrılıyor.
     */
    public void initRotationOffset() {
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /**
     * Tekerleğin şu anki açısı, radyan cinsinden. Kalibrasyon offset'i düşülmüş
     * olduğu için düz ileri 0 okunur.
     */
    public double getCANCoderRad() {
      double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
      double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
      return angle % (2 * Math.PI);
    }

    /**
     * Rotation motorunu PID ile verilen açıya çevirir.
     *
     * @param angleRad hedef açı (radyan)
     */
    public void setRotationMotorAnglePID(double angleRad) {
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), angleRad));
    }

    /**
     * Encoder'ları sıfırlar. Drive encoder 0 olur, mesafe baştan ölçülmeye başlar.
     * Rotation encoder da CANcoder'ın şu anki açısına eşitlenir.
     */
    public void resetEncoder() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /**
     * 
     * @return drive hızı (m/s)
     */
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    /**
     * Rotation motorunun hızı, rad/s olarak.
     *
     * @return rotation hızı (rad/s)
     */
    public double getRotationVelocity(){
      return rotationEncoder.getVelocity();
    }

    /**
     * Drive ve rotation motorlarını durdurur.
     */
    public void stop(){
      driveMotor.stopMotor();
      rotationMotor.stopMotor();
    }

    /**
     * Drive motorunun konumu, metre olarak (şimdiye kadar gidilen mesafe).
     *
     * @return gidilen mesafe (metre)
     */
    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }

    /**
     * Tekerleğin şu anki direksiyon açısı, radyan olarak. Mutlak encoder'dan
     * okunduğu için zamanla kaymaz.
     *
     * @return rotation konumu (radyan)
     */
    public double getRotationPosition() {
      return getCANCoderRad();
    }

    @Override
    public void periodic() {
      // Her scheduler turunda çalışır
    }
}
