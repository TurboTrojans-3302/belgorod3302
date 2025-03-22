// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Add your docs here. */
public class IntegerChange implements BooleanSupplier {

    private Supplier<Integer> supplier;
    private int lastValue;

    public IntegerChange(Supplier<Integer> supplier) {
        this.supplier = supplier;
        lastValue = supplier.get();
    }

    @Override
    public boolean getAsBoolean() {
        int value = supplier.get();
        if (lastValue != value) {
            lastValue = value;
            return true;
        }
        return false;
    }
}
