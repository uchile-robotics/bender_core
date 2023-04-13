# Agregar voz en español

### Ref: https://www.voztovoice.org/?q=node/97 

Es necesario descargar e instalar la voz en español siguiendo las siguientes instrucciones:
#### Con comandos:
1. `cd /usr/share/festival/voices`
2. `wget http://www.voztovoice.org/tmp/festival-spanish.zip`
3. `yum install unzip`
4. `unzip festival-spanish.zip`
5. `nano /usr/share/festival/festival.scm`
6. Agregar las siguientes líneas

#### Con pocos comandos:
1. Descargar voz desde http://www.voztovoice.org/tmp/festival-spanish.zip
2. Descomprimir y mover a /usr/share/festival/voices
3. Ejecutar `nano /usr/share/festival/festival.scm` y agregar las siguiente líneas:

```
;(language__spanish)
(set! voice_default 'voice_el_diphone)

    (define (tts_textasterisk string mode)
    "(tts_textasterisk STRING MODE)
    Apply tts to STRING. This function is specifically designed for
    use in server mode so a single function call may synthesize the string.
    This function name may be added to the server safe functions."
    (let ((wholeutt (utt.synth (eval (list 'Utterance 'Text string)))))
    (utt.wave.resample wholeutt 8000)
    (utt.wave.rescale wholeutt 5)
    (utt.send.wave.client wholeutt)))
```

4. Guardar y listo:)