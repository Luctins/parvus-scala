# parvus-scala
Projeto da disciplina de PIV da engenharia mecatrônica do IFSC Cãmpus Florianópolis

# How to run
 call `python3 soft_plc.py -h` for more information

# Future improvements
- implement async read and write to registers, also pack read and writes to get beter efficiency
- implement controller disable with auto mode
- Don't initialize dicts every loop, set them in the beggining and read from self
- some way to deal with multi register (and coil) writes in soft_plc (value would be an array)
