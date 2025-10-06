#!/bin/bash
# ==================================================
# Script de bash para las actividades de la Práctica 1
# ==================================================

# Despliega un mensaje
echo "Hola, esto es un mensaje de inicio para la Práctica 1"

# Mueve la ruta a la ruta del usuario
cd /home/robousr/ || { echo "No se puede encontrar la ruta"; exit 1; }

# Crea las carpetas
mkdir -p /home/robousr/Practica1
mkdir -p /home/robousr/Practica1/Letras
mkdir -p /home/robousr/Practica1/Letras/Integrantes

# Crea los archivos correspondientes
touch /home/robousr/Practica1/Letras/a.txt /home/robousr/Practica1/Letras/b.txt /home/robousr/Practica1/Letras/c.txt 
touch /home/robousr/Practica1/Letras/Integrantes/Bruno.txt /home/robousr/Practica1/Letras/Integrantes/Roberto.txt /home/robousr/Practica1/Letras/Integrantes/Arturo.txt

# Muestra un diagrama de la estructura de los archivos y carpetas
tree Practica1

rm -rf Practica1