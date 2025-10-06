
class Calculadora:
    """
    Una calculadora simple que opera con dos números.
    """
    def suma(self, a, b):
        """
        Regresa la suma de dos números (a + b).
        """
        return a + b

    def multiplicacion(self, a, b):
        """
        Regresa la multiplicación de dos números (a * b).
        """
        return a * b

class CalculadoraMultiple:
    """
    Una calculadora que puede operar con cualquier cantidad de números.
    Utiliza *numeros para aceptar un número variable de argumentos.
    """
    def suma(self, *numeros):
        """
        Regresa la suma de todos los números pasados como parámetros.
        """
        resultado = 0
        for numero in numeros:
            resultado += numero
        return resultado

    def multiplicacion(self, *numeros):
        """
        Regresa la multiplicación de todos los números pasados como parámetros.
        """
        # Se inicia en 1 porque cualquier número multiplicado por 0 es 0.
        resultado = 1
        for numero in numeros:
            resultado *= numero
        return resultado
    
#Calculadora simple
print("Probando la Calculadora (con dos parámetros):")
calc_simple = Calculadora()
suma_simple = calc_simple.suma(10, 5)

print(f"Suma: 10 + 5 = {suma_simple}")
multiplicacion_simple = calc_simple.multiplicacion(10, 5)

print(f"Multiplicación: 10 * 5 = {multiplicacion_simple}")
print("\n" + "="*30 + "\n") # Separador visual

#Calculadora multiple
print("Probando la CalculadoraMultiple (con múltiples parámetros):")
calc_multi = CalculadoraMultiple()

suma_multi = calc_multi.suma(1, 2, 3, 4, 5)
print(f"Suma: 1 + 2 + 3 + 4 + 5 = {suma_multi}") # Salida esperada: 15


multiplicacion_multi = calc_multi.multiplicacion(2, 3, 4)
print(f"Multiplicación: 2 * 3 * 4 = {multiplicacion_multi}") # Salida esperada: 24