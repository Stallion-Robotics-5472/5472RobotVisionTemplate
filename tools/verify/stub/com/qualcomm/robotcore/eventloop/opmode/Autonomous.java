package com.qualcomm.robotcore.eventloop.opmode;
import java.lang.annotation.*;
@Retention(RetentionPolicy.SOURCE) @Target(ElementType.TYPE)
public @interface Autonomous { String name() default ""; String group() default ""; }
