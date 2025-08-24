package lib.ironpulse.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.LoggedTunableNumber;

/**
 * Builder for creating named superstructure poses with dynamic parameters.
 * Allows adding arbitrary named parameters that can be tuned via NetworkTables.
 */
public class SuperstructurePoseBuilder {
    private final String name;
    private final Map<String, DoubleSupplier> parameters = new HashMap<>();
    
    private SuperstructurePoseBuilder(String name) {
        this.name = name;
    }
    
    /**
     * Creates a new pose builder with the given name.
     * @param name The name of this pose (used for NetworkTables tuning)
     * @return A new SuperstructurePoseBuilder instance
     */
    public static SuperstructurePoseBuilder create(String name) {
        return new SuperstructurePoseBuilder(name);
    }
    
    /**
     * Adds a parameter with a fixed value.
     * @param parameterName The name of the parameter
     * @param value The fixed value for this parameter
     * @return This builder for chaining
     */
    public SuperstructurePoseBuilder withParameter(String parameterName, double value) {
        return withParameter(parameterName, new LoggedTunableNumber("Superstructure/" + name + "/" + parameterName, value));
    }
    
    /**
     * Adds a parameter with a dynamic supplier.
     * @param parameterName The name of the parameter
     * @param supplier The supplier providing dynamic values
     * @return This builder for chaining
     */
    public SuperstructurePoseBuilder withParameter(String parameterName, DoubleSupplier supplier) {
        parameters.put(parameterName, supplier);
        return this;
    }
    
    /**
     * Builds the final SuperstructurePose.
     * @return A new SuperstructurePose instance
     */
    public SuperstructurePose build() {
        return new SuperstructurePose(name, parameters);
    }
}