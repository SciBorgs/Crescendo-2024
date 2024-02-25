package org.sciborgs1155.lib;

import java.util.Optional;

public final class IHateThisLanguage {
  @FunctionalInterface
  public interface ExceptionSupplier<T> {
    public T get() throws Exception;
  }

  public static <T> Optional<T> handle(ExceptionSupplier<T> wtf) {
    try {
      return Optional.of(wtf.get());
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
