package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Swerve sürüş sistemi. Robotun nasıl hareket edeceğine bu class karar veriyor.
 *
 * Dört tekerleğin her biri hem istediği yöne bakabiliyor hem istediği hızda dönebiliyor.
 * Bu sayede robot her yöne gidebiliyor
 *
 * Class'ın yönettikleri:
 *    Dört SwerveModule 
 *    robotun hangi yöne baktığını söyler
 *    robotun sahadaki yerini tahmin eder. Tekerlek encoder'ı,
 *     gyro ve Limelight verisini birleştirir
 * AprilTag'leri okur, hem nişan almak hem konum bulmak için kullanılır
 *  auto sırasında robotu çizilmiş path'ler üzerinde sürer
 *
 * periodic() her 20 ms'de bir konum tahminini hem encoder'larla hem de Limelight ile
 * günceller. İkisini birlikte kullanmak, tek başına birini kullanmaktan daha doğru sonuç veriyor.
 *
 * Sürüşü değiştirecekseniz drive() metoduna bakın. Kamerayla ilgili işler
 * updateVisionPose() içinde. Auto path takibi ise constructor'daki AutoBuilder.configure()
 * çağrısında ayarlanıyor.
 */
public class SwerveSubsystem extends SubsystemBase{

    /**
     * Sahaya göre mi (true), robota göre mi (false) sürüyoruz?
     * Field-relative'de joystick'i ileri itince robot, burnu nereye bakıyor olursa
     * olsun karşı duvara gider. Robot-relative'de ise burnunun baktığı yöne gider.
     */
    public static boolean fieldRelativeStatus = true;

   
    private final SwerveModule frontLeft = new SwerveModule(
        DrivetrainConstants.kFLDrive,
        DrivetrainConstants.kFLRotate,
        DrivetrainConstants.kFLCanCoder,
        DrivetrainConstants.kFLOffsetRad,
        DrivetrainConstants.fLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule frontRight = new SwerveModule(
        DrivetrainConstants.kFRDrive,
        DrivetrainConstants.kFRRotate,
        DrivetrainConstants.kFRCanCoder,
        DrivetrainConstants.kFROffsetRad,
        DrivetrainConstants.fRIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backLeft = new SwerveModule(
        DrivetrainConstants.kBLDrive,
        DrivetrainConstants.kBLRotate,
        DrivetrainConstants.kBLCanCoder,
        DrivetrainConstants.kBLOffsetRad,
        DrivetrainConstants.bLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backRight = new SwerveModule(
        DrivetrainConstants.kBRDrive,
        DrivetrainConstants.kBRRotate,
        DrivetrainConstants.kBRCanCoder,
        DrivetrainConstants.kBROffsetRad,
        DrivetrainConstants.bRIsInverted,
        DrivetrainConstants.kPRotation
    );

    // ========================
    // SENSÖRLER
    // ========================

    /** NavX gyro. Robotun baktığı yönü ölçer. */
    private final AHRS navX;

    /**
     * Robotun sahadaki konumunu tutar. Tekerlek hareketi, gyro yönü ve Limelight
     * okumalarını birleştirir. Sadece tekerlek dönüşünü saymaktan daha doğru çalışır,
     * çünkü kamera zamanla biriken hatayı düzeltir.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** Limelight'ın NetworkTables bağlantısı. Hedef verisinin tamamı buradan okunuyor. */
    private final NetworkTable limelightTable;



    /**
     * Constructor: donanımı kurar, PathPlanner'ı ayarlar.
     *
     * NavX reset'i ayrı bir thread'de, 1 saniye bekledikten sonra yapılıyor. NavX
     * güç alınca açılması yaklaşık bir saniye sürüyor, hemen reset atarsak hazır
     * olmayabilir.
     */
    public SwerveSubsystem(){
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // NavX açılırken iç sensörlerini kalibre ediyor. Reset'i bu yüzden bir saniye geciktiriyoruz.
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX.reset();
            }
            catch(Exception e){
                e.printStackTrace();
            }
        }).start();

        // Motor encoder'larını CANcoder'lara eşitliyoruz.
        // CANcoder tekerlek açısını reboot sonrası bile bilir. Motorun encoder'ı ise
        // güç gidince unutur. Burada gerçek açıyı motor encoder'ına yazıyoruz,
        // ikisi aynı değeri göstersin.
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        // Drive encoder'ları sıfırla, mesafe buradan itibaren ölçülecek.
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        // --- Limelight ---
        // Limelight'a bağlanıp ne gördüğünü okuyabilmek için tabloyu alıyoruz.
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);

        // Kamera görüntüsünü Shuffleboard'da "Driver" sekmesine koyuyoruz,
        // sürücüler de ne gördüğünü izleyebilsin.
        HttpCamera cameraStream = new HttpCamera("LimelightStream",
            "http://" + VisionConstants.kLimelightName + ".local:5800/stream.mjpg");

        Shuffleboard.getTab("Driver")
            .add("Limelight", cameraStream)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(4, 3);

        // --- Pose estimator ---
        // Başlangıç noktası (0,0) ve robot ileri bakıyor (0 derece).
        // Bu değer her 20 ms'de encoder ve kamera verisiyle düzeltilecek.
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SwerveDriveKinematics,
            new Rotation2d(), // Başlangıç gyro açısı
            getModulePositions(),
            new Pose2d() // Başlangıç konumu (0, 0, 0 derece)
        );

        // --- PathPlanner ---
        // Robot ayarlarını PathPlanner GUI'sinin kaydettiği dosyadan okuyoruz.
        // Kütle, atalet momenti gibi fiziksel değerler orada duruyor. PathPlanner
        // düzgün path üretmek için bunlara ihtiyaç duyar.
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
        }

        // AutoBuilder'ı ayarlıyoruz, auto'da robotu sürebilsin.
        // Ona şunları veriyoruz: konumu nereden okuyacak, nasıl sıfırlayacak,
        // şu anki hızı nereden alacak ve robotu nasıl sürecek.
        AutoBuilder.configure(
            this::getPose, // Robotun şu anki konumu
            this::resetOdometry, // Konumu sıfırlama (auto başında kullanılır)
            this::getRobotChassisSpeeds, // Şu anki hız (robot-relative olmak zorunda)
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Robotu sürme
            new PPHolonomicDriveController( // Path takibi için PID
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID (konuma gitme)
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (açıya dönme)
            ),
            config, // Robotun fiziksel ayarları
            () -> {
              // Path'ler kırmızı ittifakta aynalansın mı?
              // Path'leri mavi taraf için çiziyoruz, kırmızıdaysak ters çeviriyoruz.
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Auto sırasında PathPlanner bu subsystem'i require edecek
        );
    }


    /**
     * Dört modülün hem drive hem rotation motorlarını durdurur.
     */
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Robotun sahadaki tahmini konumu: x, y ve yön.
     */
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Robotun şu anki hareket hızı ve dönüş hızı. Tekerleklerin gerçek hızlarından hesaplanır.
     */
    public ChassisSpeeds getRobotChassisSpeeds(){
        return DrivetrainConstants.SwerveDriveKinematics.toChassisSpeeds(getStates());
    }

    /**
     * Robotu verilen hızlarda sürer. Auto'da PathPlanner çağırıyor.
     * Hiçbir tekerlek max hızı aşmasın diye hızlar sınırlanıyor.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        setModuleStates(moduleStates);
    }

    /**
     * Robotun baktığı yön, NavX'ten okunuyor. Başına eksi koyduk, çünkü NavX ile
     * WPILib hangi yönün pozitif olduğunda ters düşüyor.
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    /** NavX nesnesini olduğu gibi verir. Doğrudan erişmek gerekirse kullan. */
    public AHRS getNavX(){
        return navX;
    }

    /**
     * Gyro'yu sıfırlar. Robotun o an baktığı yön "ileri" (0 derece) kabul edilir.
     * Maç başında ya da field-relative sürüş yamuk gelmeye başladığında kullan.
     */
    public void resetHeading(){
        navX.reset();
    }

    /**
     * Robota sahadaki yerini söyler. Auto başında başlangıç konumunu vermek için kullanılıyor.
     */
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /**
     * Her tekerleğe hedef hızını ve açısını verir.
     * Hedef açıları ayrıca dashboard'a da yazıyor, yanlış giden bir şey olursa görürsün.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);

        // Verilen açıları debug için dashboard'a yaz
        SmartDashboard.putNumber("FL Set Position", desiredStates[0].angle.getRadians());
        SmartDashboard.putNumber("FR Set Position", desiredStates[1].angle.getRadians());
        SmartDashboard.putNumber("BL Set Position", desiredStates[2].angle.getRadians());
        SmartDashboard.putNumber("BR Set Position", desiredStates[3].angle.getRadians());
    }

    /**
     * Her modülün şu anki konumu: gidilen mesafe ve açı.
     * Pose estimator, robotun nasıl hareket ettiğini bunlarla izler.
     *
     * @return [FL, FR, BL, BR] sırasıyla 4 SwerveModulePosition
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad())),
        };
        return positions;
    }

    /**
     * Her modülün şu anki durumu: hız ve açı.
     * Kinematik hesaplarında ve loglamada kullanılıyor.
     *
     * @return [FL, FR, BL, BR] sırasıyla 4 SwerveModuleState
     */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    /**
     * Sürüşün field-relative ya da robot-relative olmasını seçer.
     *
     * @param relativity true: field-relative, false: robot-relative
     */
    public void setFieldRelativity(boolean relativity){
        fieldRelativeStatus = relativity;
    }

    /**
     * Ana sürüş metodu. Robotu hareket ettiren asıl yer burası.
     *
     * İleri, yan ve dönüş hızını alır, her tekerleğin ne yapacağını hesaplar.
     * Field-relative ve robot-relative modların ikisinde de çalışır.
     *
     * @param forward  ileri/geri hız, m/s (pozitif ileri)
     * @param strafe   sağa/sola hız, m/s (pozitif sol)
     * @param rotation dönüş hızı, rad/s (pozitif saat yönünün tersi)
     * @param isFieldRelative true ise field-relative, false ise robot-relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative){
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);

        // Hem giderken hem dönerken robot yana doğru sürüklenir. Hesap yapılırken
        // robot da hareket ettiği için oluyor bu. discretize bunu düzeltiyor.
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Hızı kaydediyoruz, sonra tekrar oynatıp inceleyebilmek için
        Logger.recordOutput("ChassisSpeeds", speeds);

        // Her tekerleğin yapacağını hesapla, hiçbiri max hızı geçmesin
        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);
        setModuleStates(states);

        // Verilen hızları debug için dashboard'a yaz
        SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
    }

    /**
     * Her 20 ms'de kendiliğinden çalışır. Konumu tekerlek, gyro ve kamera verisiyle
     * günceller, ardından debug bilgilerini dashboard'a gönderir.
     */
    @Override
    public void periodic(){
        super.periodic();

        // Tekerlek ve gyro ile konum güncelle (iyi çalışır ama zamanla kayar)
        poseEstimator.update(getHeading(), getModulePositions());

        // Limelight ile konum güncelle (yukarıdaki kaymayı düzeltir)
        updateVisionPose();

        // AdvantageKit için log
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());

        // Debug verisini yayınla
        execute();
    }

    /**
     * Limelight'tan gelen veriyle robotun konumunu düzeltir.
     *
     * Kamera AprilTag'lere bakıp robotun nerede olduğunu hesaplıyor. Biz de x, y ve
     * yönü alıp estimator'a veriyoruz. Fotoğraf birkaç milisaniye önce çekildiği
     * için o gecikmeyi zaman damgasından çıkarıyoruz.
     *
     * Hedef görünmüyorsa ya da veri bozuksa hiçbir şey yapmadan çıkar.
     */
    private void updateVisionPose() {
        // tv = "target valid". 1.0 ise Limelight bir AprilTag görüyor.
        if (limelightTable.getEntry("tv").getDouble(0) != 1.0) return;

        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (botpose.length < 7) return;

        // Limelight arada bozuk veri gönderebiliyor, NaN ya da sonsuz değer varsa atla
        for (double v : botpose) {
            if (Double.isNaN(v) || Double.isInfinite(v)) return;
        }

        // Vision verisinden (x, y, yaw) bir Pose2d yap
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));

        // Kameranın gecikmesini düş, görüntü birkaç ms önce alınmıştı
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);

        // Ölçümü pose estimator'a ver
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /**
     * Limelight geçerli bir AprilTag görüyorsa true döner.
     *
     * @return hedef görünüyorsa true
     */
    public boolean getLimelightHasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    /**
     * Hedefin kamera merkezine göre yatay sapması, derece cinsinden.
     * Negatifse hedef solda, pozitifse sağda. AlignToTag command'ı hedefe
     * otomatik dönerken bunu kullanıyor.
     */
    public double getLimelightTx() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * Kameranın baktığı AprilTag'in numarası. Hiçbir tag görünmüyorsa -1.
     * Auto-shooter, kendi ittifakımızın hedefine nişan aldığından emin olmak için bunu kullanıyor.
     */
    public int getLimelightTid() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Hedefe olan düz çizgi mesafe, metre cinsinden. Kameranın 3 boyutlu verisinden
     * hesaplanıyor. Hedef görünmüyorsa -1.0 döner.
     */
    public double getLimelightTargetDistanceMeters() {
        double[] targetpose = limelightTable.getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[0]);
        if (targetpose.length < 3) return -1.0;
        return Math.sqrt(targetpose[0] * targetpose[0]
            + targetpose[1] * targetpose[1]
            + targetpose[2] * targetpose[2]);
    }

    /**
     * Encoder ve sensör değerlerini debug için SmartDashboard'a yazar.
     * periodic() her 20 ms'de bunu çağırıyor.
     */
    protected void execute() {
        // Drive encoder hızları (m/s)
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        // Direksiyon encoder konumları (radyan)
        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        // NavX yönü (derece)
        SmartDashboard.putNumber("Yaw", -navX.getYaw());

        // Şu anki field-relative durumu
        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);
    }
}
