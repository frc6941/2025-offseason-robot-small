package lib.ntext;

import com.google.auto.service.AutoService;

import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.util.ElementFilter;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;
import java.io.IOException;
import java.io.Writer;
import java.util.*;

@SupportedAnnotationTypes({"lib.ntext.NTParameter"})
@SupportedSourceVersion(SourceVersion.RELEASE_17)
@SupportedOptions("DEBUG")
@AutoService(Processor.class)
public class NTParameterProcessor extends AbstractProcessor {
    private static final Map<String, GetterSetterPair> NT_TYPES_TABLE = Map.ofEntries(
            // Booleans
            Map.entry("boolean", GetterSetterPair.of("getBoolean", "setBoolean")),
            Map.entry("java.lang.Boolean", GetterSetterPair.of("getBoolean", "setBoolean")),
            Map.entry("boolean[]", GetterSetterPair.of("getBooleanArray", "setBooleanArray")),
            Map.entry("java.lang.Boolean[]", GetterSetterPair.of("getBooleanArray", "setBooleanArray")),

            // Integers
            Map.entry("int", GetterSetterPair.of("getInteger", "setInteger")),
            Map.entry("java.lang.Integer", GetterSetterPair.of("getInteger", "setInteger")),
            Map.entry("int[]", GetterSetterPair.of("getIntegerArray", "setIntegerArray")),
            Map.entry("java.lang.Integer[]", GetterSetterPair.of("getIntegerArray", "setIntegerArray")),

            Map.entry("long", GetterSetterPair.of("getInteger", "setInteger")),
            Map.entry("java.lang.Long", GetterSetterPair.of("getInteger", "setInteger")),
            Map.entry("long[]", GetterSetterPair.of("getIntegerArray", "setIntegerArray")),
            Map.entry("java.lang.Long[]", GetterSetterPair.of("getIntegerArray", "setIntegerArray")),

            // Floats
            Map.entry("float", GetterSetterPair.of("getFloat", "setFloat")),
            Map.entry("java.lang.Float", GetterSetterPair.of("getFloat", "setFloat")),
            Map.entry("float[]", GetterSetterPair.of("getFloatArray", "setFloatArray")),
            Map.entry("java.lang.Float[]", GetterSetterPair.of("getFloatArray", "setFloatArray")),

            // Doubles
            Map.entry("double", GetterSetterPair.of("getDouble", "setDouble")),
            Map.entry("java.lang.Double", GetterSetterPair.of("getDouble", "setDouble")),
            Map.entry("double[]", GetterSetterPair.of("getDoubleArray", "setDoubleArray")),
            Map.entry("java.lang.Double[]", GetterSetterPair.of("getDoubleArray", "setDoubleArray")),

            // Strings
            Map.entry("java.lang.String", GetterSetterPair.of("getString", "setString")),
            Map.entry("java.lang.String[]", GetterSetterPair.of("getStringArray", "setStringArray")),

            // Raw bytes
            Map.entry("byte[]", GetterSetterPair.of("getRaw", "setRaw"))
    );

    public static Boolean isTuning = false;

    /**
     * Handle processor options.
     */
    private void handleTuning(Element element) {
        // annotation attribute takes highest precedence
        NTParameter annotation = element.getAnnotation(NTParameter.class);
        if (annotation != null && annotation.isTuning()) {
            isTuning = true;
            return;
        }
        // next, processor option -ADEBUG=true
        String opt = processingEnv.getOptions().get("DEBUG");
        if (opt != null) {
            isTuning = Boolean.parseBoolean(opt);
            return;
        }
        // finally, system property passed to compiler JVM
        if (isTuning == null) {
            isTuning = Boolean.getBoolean("lib.ntext.ntparameter.debug");
        }
    }

    /**
     * Main processing method for dealing with all the annotations.
     *
     * @param annotations set all annotated classes.
     * @param roundEnv    processing env for the processor.
     * @return if the processor runs successfully.
     */
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        // Group annotated elements by their containing class
        Map<TypeElement, List<VariableElement>> classToFields = new HashMap<>();
        Set<TypeElement> annotatedClasses = new HashSet<>();

        for (Element annotated : roundEnv.getElementsAnnotatedWith(NTParameter.class)) {
            if (annotated.getKind() == ElementKind.CLASS) {
                // Class-level annotation
                TypeElement rootClass = (TypeElement) annotated;
                annotatedClasses.add(rootClass);
                handleTuning(rootClass);
                validateFields(rootClass);
                generateWrapper(rootClass);
            } else if (annotated.getKind() == ElementKind.FIELD) {
                // Field-level annotation
                VariableElement field = (VariableElement) annotated;
                TypeElement containingClass = (TypeElement) field.getEnclosingElement();

                // Validate the field
                if (!validateSingleField(field)) {
                    continue;
                }

                handleTuning(field);

                // Group fields by their containing class
                classToFields.computeIfAbsent(containingClass, k -> new ArrayList<>()).add(field);
            } else {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "@lib.ntext.NTParameter can only be applied to classes or fields.", annotated
                );
            }
        }

        // Process field-level annotations grouped by class
        for (Map.Entry<TypeElement, List<VariableElement>> entry : classToFields.entrySet()) {
            TypeElement containingClass = entry.getKey();
            List<VariableElement> fields = entry.getValue();

            // Skip if this class was already processed as a class-level annotation
            if (annotatedClasses.contains(containingClass)) {
                continue;
            }

            generateFieldWrapper(containingClass, fields);
        }

        return true;
    }

    /**
     * Validate a single field for field-level annotations.
     */
    private boolean validateSingleField(VariableElement field) {
        String className = field.getEnclosingElement().getSimpleName().toString();

        if (!field.getModifiers().contains(Modifier.STATIC)) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "Field '" + field.getSimpleName() + "' in class '" + className + "' must be declared static.",
                    field
            );
            return false;
        }

        String typeName = field.asType().toString();
        if (!NT_TYPES_TABLE.containsKey(typeName)) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "Invalid field type '" + typeName + "' in class '" + className + "'.",
                    field
            );
            return false;
        }

        return true;
    }

    /**
     * Recursive method to check all the fields of a class.
     *
     * @param classElement root class.
     */
    private void validateFields(TypeElement classElement) {
        String className = classElement.getQualifiedName().toString();
        List<VariableElement> fields = ElementFilter.fieldsIn(classElement.getEnclosedElements());
        for (VariableElement field : fields) {
            if (!field.getModifiers().contains(Modifier.STATIC)) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "Field '" + field.getSimpleName() + "' in class '" + className + "' must be declared static.",
                        field
                );
                continue;
            }

            String typeName = field.asType().toString();
            if (!NT_TYPES_TABLE.containsKey(typeName)) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "Invalid field type '" + typeName + "' in class '" + className + "'.",
                        field
                );
                throw new FieldTypeError("Invalid field type '" + typeName + "' in class '" + className + "'.");
            }
        }

        for (Element enclosed : classElement.getEnclosedElements()) {
            if (enclosed.getKind() == ElementKind.CLASS && enclosed.getModifiers().contains(Modifier.STATIC)) {
                validateFields((TypeElement) enclosed);
            } else if (enclosed.getKind() == ElementKind.CLASS) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "Nested class '" + enclosed.getSimpleName() + "' in '" + className + "' must be declared static.",
                        enclosed
                );
                throw new FieldTypeError("Nested class '" + enclosed.getSimpleName() + "' in '" + className + "'.");
            }
        }
    }

    /**
     * Generate wrapper for field-level annotations.
     */
    private void generateFieldWrapper(TypeElement containingClass, List<VariableElement> annotatedFields) {
        // Use the first field's annotation for table name, or derive from class name
        String tableName = null;
        for (VariableElement field : annotatedFields) {
            NTParameter annotation = field.getAnnotation(NTParameter.class);
            if (annotation != null && !annotation.tableName().isEmpty()) {
                tableName = annotation.tableName();
                break;
            }
        }

        if (tableName == null || tableName.isEmpty()) {
            tableName = containingClass.getSimpleName().toString();
        }

        String pkgName = processingEnv.getElementUtils().getPackageOf(containingClass).toString();
        String className = containingClass.getSimpleName() + "NT";

        // start builder, write header
        StringBuilder builder = new StringBuilder();
        builder.append("package ").append(pkgName).append(";\n\n")
                .append("import edu.wpi.first.networktables.NetworkTableEntry;\n")
                .append("import edu.wpi.first.networktables.NetworkTableInstance;\n\n")
                .append("public class ").append(className).append(" {\n");

        // Process only the annotated fields
        buildFieldsContent(annotatedFields, tableName, builder, "  ");

        // end, write as a generated java file
        builder.append("}\n");
        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(pkgName + "." + className, containingClass);
            try (Writer writer = file.openWriter()) {
                writer.write(builder.toString());
            }
        } catch (IOException e) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR, "Codegen failed: " + e.getMessage(), containingClass);
        }
    }

    /**
     * Build content for specific fields (used for field-level annotations).
     */
    private void buildFieldsContent(List<VariableElement> fields, String tableName, StringBuilder builder,
                                    String indent) {
        String tableChain = ".getTable(\"" + tableName + "\")";

        for (VariableElement field : fields) {
            String fieldName = field.getSimpleName().toString();
            String typeName = field.asType().toString();
            String getterName = NT_TYPES_TABLE.get(typeName).getGetter();
            String setterName = NT_TYPES_TABLE.get(typeName).getSetter();
            Object defaultValue = field.getConstantValue();

            String defaultLiteral = getDefaultLiteral(defaultValue, fieldName, getterName, typeName);

            if (!isTuning) {
                builder.append(indent).append("public static ").append(typeName).append(" ")
                        .append(fieldName).append("() {\n")
                        .append(indent).append("  return ").append(defaultLiteral).append(";\n")
                        .append(indent).append("}\n");
            } else {
                // add getter with special handling for integers
                if ("int".equals(typeName) || "java.lang.Integer".equals(typeName)) {
                    builder.append(indent).append("public static int ").append(fieldName).append("() {\n")
                            .append(indent).append("  return Math.toIntExact(NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".getInteger(").append(defaultLiteral).append("L)") // append L for long literal
                            .append(");\n")
                            .append(indent).append("}\n");
                } else if ("int[]".equals(typeName) || "java.lang.Integer[]".equals(typeName)) {
                    builder.append(indent).append("public static int[] ").append(fieldName).append("() {\n")
                            .append(indent).append("  long[] longs = NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".getIntegerArray(new long[0]);\n")
                            .append(indent).append("  int[] ints = new int[longs.length];\n")
                            .append(indent).append(
                                    "  for (int i = 0; i < longs.length; ++i) ints[i] = Math.toIntExact(longs[i]);\n")
                            .append(indent).append("  return ints;\n")
                            .append(indent).append("}\n");
                } else {
                    builder.append(indent).append("public static ").append(typeName).append(" ")
                            .append(fieldName).append("() {\n")
                            .append(indent).append("  return NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".").append(getterName).append("(").append(defaultLiteral).append(");\n")
                            .append(indent).append("}\n");
                }

                // add static setter
                builder.append(indent).append("static {\n");
                builder.append(indent).append("  NetworkTableInstance.getDefault()")
                        .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                        .append(".").append(setterName).append("(").append(defaultLiteral).append(");\n");
                builder.append(indent).append("}\n");
            }
        }
    }

    /**
     * Entry method for generating corresponding network table method for the class.
     *
     * @param rootClass root class.
     */
    private void generateWrapper(TypeElement rootClass) {
        String tableName = rootClass.getAnnotation(NTParameter.class).tableName();
        String pkgName = processingEnv.getElementUtils().getPackageOf(rootClass).toString();
        String className = rootClass.getSimpleName() + "NT";

        // start builder, write header
        StringBuilder builder = new StringBuilder();
        builder.append("package ").append(pkgName).append(";\n\n")
                .append("import edu.wpi.first.networktables.NetworkTableEntry;\n")
                .append("import edu.wpi.first.networktables.NetworkTableInstance;\n\n")
                .append("public class ").append(className).append(" {\n");

        // do recursive adding
        buildClassContent(rootClass, tableName, builder, "  ", "");

        // end, write as a generated java file
        builder.append("}\n");
        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(pkgName + "." + className, rootClass);
            try (Writer writer = file.openWriter()) {
                writer.write(builder.toString());
            }
        } catch (IOException e) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR, "Codegen failed: " + e.getMessage(), rootClass);
        }
    }

    /**
     * Get default literal value for a field.
     */
    private String getDefaultLiteral(Object defaultValue, String fieldName, String getterName, String typeName) {
        if (defaultValue != null) {
            if (defaultValue instanceof String s) {
                return "\"" + s.replace("\"", "\\\"") + "\"";
            } else if (defaultValue instanceof Character c) {
                return "'" + c + "'";
            } else {
                return defaultValue.toString();
            }
        } else {
            // Fallback: use field name as string default for getString or just empty/zero fallback
            if (getterName.equals("getString")) {
                return "\"" + fieldName + "\"";
            } else if (getterName.startsWith("getBoolean")) {
                return "false";
            } else if (getterName.startsWith("getInteger")) {
                return "0";
            } else if (getterName.startsWith("getFloat")) {
                return "0.0f";
            } else if (getterName.startsWith("getDouble")) {
                return "0.0";
            } else if (getterName.endsWith("Array")) {
                return "new " + typeName.replace("[]", "") + "[0]";
            } else if (getterName.equals("getRaw")) {
                return "new byte[0]";
            } else {
                return "null"; // fallback fallback
            }
        }
    }

    /**
     * Recursive method for adding field wrapper methods to the java file.
     *
     * @param classElement current class.
     * @param tablePath    current nt table path.
     * @param builder      string builder.
     * @param indent       current indentation.
     * @param prefix       current table prefix.
     */
    private void buildClassContent(TypeElement classElement, String tablePath, StringBuilder builder, String indent,
                                   String prefix) {
        List<VariableElement> fields = ElementFilter.fieldsIn(classElement.getEnclosedElements());

        for (VariableElement field : fields) {
            String fieldName = field.getSimpleName().toString();
            String tableChain = buildTableChain(tablePath, prefix);
            String typeName = field.asType().toString();
            String getterName = NT_TYPES_TABLE.get(typeName).getGetter();
            String setterName = NT_TYPES_TABLE.get(typeName).getSetter();
            Object defaultValue = field.getConstantValue();

            String defaultLiteral = getDefaultLiteral(defaultValue, fieldName, getterName, typeName);

            if (!isTuning) {
                builder.append(indent).append("public static ").append(typeName).append(" ")
                        .append(fieldName).append("() {\n")
                        .append(indent).append("  return ").append(defaultLiteral).append(";\n")
                        .append(indent).append("}\n");
            } else {
                // add getter
                // NOTE: special treatment for integer and related types are required, as NT4 only returns long
                if ("int".equals(typeName) || "java.lang.Integer".equals(typeName)) {
                    builder.append(indent).append("public static int ").append(fieldName).append("() {\n")
                            .append(indent).append("  return Math.toIntExact(NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".getInteger(").append(defaultLiteral).append("L)") // append L for long literal
                            .append(");\n")
                            .append(indent).append("}\n");
                } else if ("int[]".equals(typeName) || "java.lang.Integer[]".equals(typeName)) {
                    builder.append(indent).append("public static int[] ").append(fieldName).append("() {\n")
                            .append(indent).append("  long[] longs = NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".getIntegerArray(new long[0]);\n")
                            .append(indent).append("  int[] ints = new int[longs.length];\n")
                            .append(indent).append(
                                    "  for (int i = 0; i < longs.length; ++i) ints[i] = Math.toIntExact(longs[i]);\n")
                            .append(indent).append("  return ints;\n")
                            .append(indent).append("}\n");
                } else {
                    builder.append(indent).append("public static ").append(typeName).append(" ")
                            .append(fieldName).append("() {\n")
                            .append(indent).append("  return NetworkTableInstance.getDefault()")
                            .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                            .append(".").append(getterName).append("(").append(defaultLiteral).append(");\n")
                            .append(indent).append("}\n");
                }

                // add static setter
                builder.append(indent).append("static {\n");
                builder.append(indent).append("  NetworkTableInstance.getDefault()")
                        .append(tableChain).append(".getEntry(\"").append(fieldName).append("\")")
                        .append(".").append(setterName).append("(").append(defaultLiteral).append(");\n");
                builder.append(indent).append("}\n");
            }
        }

        for (Element enclosed : classElement.getEnclosedElements()) {
            if (enclosed.getKind() == ElementKind.CLASS && enclosed instanceof TypeElement nested) {
                String nestedName = nested.getSimpleName().toString();
                builder.append(indent).append("public static class ").append(nestedName).append(" {\n");
                buildClassContent(nested, tablePath, builder, indent + "  ", prefix + nestedName + ".");
                builder.append(indent).append("}\n");
            }
        }
    }

    private String buildTableChain(String rootTable, String nestedPrefix) {
        StringBuilder result = new StringBuilder();
        result.append(".getTable(\"").append(rootTable).append("\")");
        for (String part : nestedPrefix.split("\\.")) {
            if (!part.isEmpty()) {
                result.append(".getSubTable(\"").append(part).append("\")");
            }
        }
        return result.toString();
    }

    public static class AnnotationTargetError extends Error {
        public AnnotationTargetError(String message) {
            super(message);
        }
    }

    public static class FieldTypeError extends Error {
        public FieldTypeError(String message) {
            super(message);
        }
    }

    private static class GetterSetterPair {
        private final String getter;
        private final String setter;

        private GetterSetterPair(String getter, String setter) {
            this.getter = getter;
            this.setter = setter;
        }

        public static GetterSetterPair of(String getter, String setter) {
            return new GetterSetterPair(getter, setter);
        }

        public String getGetter() {
            return getter;
        }

        public String getSetter() {
            return setter;
        }
    }
}