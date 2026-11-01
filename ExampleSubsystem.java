// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Boş bir Subsystem oluşturur. Yeni subsystem yazarken bu dosyayı kopyalayıp bağalarozzzz */
  public ExampleSubsystem() {}

  /**
   * Command factory örneği. Ayrı bir Command sınıfı açmadan, komutu subsystem'in
   * içinde tek seferde tanımlamanın yolu bu.
   *
   * @return çalıştırılacak commandddd
   */
  public Command exampleMethodCommand() {
    // Komutun içeriği doğrudan burada yazılıyor.
    // runOnce bu subsystem'i kendiliğinden require eder, ayrıca belirtmen gerekmez.
    return runOnce(
        () -> {
          /* tek seferlik iş buraya */
        });
  }

  /**
   * Subsystem'in bir durumunu true/false olarak döndüren örnek metot.
   * Mesela bir limit switch ya da beam-break sensörünün okuması.
   *
   * @return sensörün ya da durumun değeri
   */
  public boolean exampleCondition() {
    // Gerçek bir sensör bağlayınca burada onu oku. Şimdilik hep false.
    return false;
  }

  @Override
  public void periodic() {
    // Scheduler her turda bunu çağırır.
  }

  @Override
  public void simulationPeriodic() {
    // Simülasyonda, her scheduler turunda çağrılır
  }
}
