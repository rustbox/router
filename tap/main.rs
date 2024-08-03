use pkt::handle_eth;
use tun_tap::Iface;

fn main() {
    let ifname = std::env::args().nth(1).unwrap_or_else(|| "tap2".to_owned());
    // NB: things will go poorly if you don't set up the interface first
    // see: #setup under https://github.com/rustbox/discussion/discussions/3#discussioncomment-9334970
    let iface = Iface::without_packet_info(&ifname, tun_tap::Mode::Tap)
        .map_err(|err| format!("couldn't open interface `{ifname}`: {err}"))
        .unwrap();

    // TODO MaybeUninit for buf
    let mut buf = [0u8; 1500 /* MTU */];
    loop {
        let n = iface.recv(&mut buf).expect("recv failed");

        let data = &mut buf[0..=n];
        println!("{:02x}", AsEach(&data));

        handle_eth(data);

        println!("-> {:02x}", AsEach(&data));
        iface.send(data).expect("send failed");
    }
}

pub struct AsEach<T: AsRef<[u8]>>(T);

impl<T: AsRef<[u8]>> std::fmt::LowerHex for AsEach<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.as_ref().iter().try_for_each(|e| u8::fmt(e, f))
    }
}
