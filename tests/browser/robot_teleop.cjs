const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const os = require('node:os');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1440, height: 960}});
    page.setDefaultTimeout(15000);
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('console', message => { if (message.type() === 'error') errors.push(message.text()); });
    await page.goto(process.env.SIMULATION_URL || 'http://127.0.0.1:8080/');
    await page.locator('#connection[data-state="live"]').waitFor();
    const canvas = page.locator('#screen canvas');
    await page.waitForFunction(() => {
      const canvas = document.querySelector('#screen canvas');
      if (!canvas || canvas.width < 300) return false;
      const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
      return data.some((value, index) => index % 4 !== 3 && value > 100);
    });
    await page.waitForTimeout(2000);
    await page.screenshot({path: path.join(os.tmpdir(), 'duke-crush-browser.png')});
    await canvas.click();
    for (const key of ['w', 's', 'a', 'd', 'q', 'e', 'ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'z', 'x']) {
      await page.keyboard.down(key);
      await page.waitForTimeout(3500);
      assert.match(await page.locator('#drive-state').textContent(), /Driving/);
      await page.keyboard.up(key);
      await page.waitForTimeout(1500);
    }
    // Chording: releasing one key should retain the other axis.
    await page.keyboard.down('w'); await page.keyboard.down('d');
    await page.waitForTimeout(1200);
    await page.keyboard.up('d'); await page.waitForTimeout(600);
    assert.match(await page.locator('#drive-state').textContent(), /Driving · W$/);
    await page.keyboard.press('Space');
    await page.keyboard.up('w');
    assert.match(await page.locator('#drive-state').textContent(), /Click the viewport/);
    await page.keyboard.down('w');
    await page.waitForTimeout(500);
    await page.locator('#stop').click();
    await page.keyboard.up('w');
    await page.locator('#surface').click(); await page.waitForTimeout(500);
    await page.locator('#underwater').click(); await page.waitForTimeout(500);
    await page.locator('#follow').click(); await page.waitForTimeout(500);
    await page.locator('#fullscreen').click();
    await page.waitForFunction(() => document.fullscreenElement?.id === 'stage');
    await page.evaluate(() => document.exitFullscreen());
    await page.locator('#reconnect').click();
    await page.locator('#connection[data-state="live"]').waitFor();
    await page.waitForTimeout(1000);
    await page.setViewportSize({width: 390, height: 844});
    await page.waitForTimeout(500);
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
    await page.screenshot({path: path.join(os.tmpdir(), 'duke-crush-mobile.png')});
    await page.setViewportSize({width: 1440, height: 960});
    await canvas.click();
    await page.keyboard.down('w'); await page.waitForTimeout(600);
    // Close the connection while a key is held; telemetry must return to zero afterward.
    await page.close();
    await new Promise(resolve => setTimeout(resolve, 1000));
    assert.deepEqual(errors, []);
    console.log('PASS: twelve motion keys, chords, Space/Stop, camera presets, fullscreen, reconnect, responsive layout, disconnect while driving');
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exit(1); });
